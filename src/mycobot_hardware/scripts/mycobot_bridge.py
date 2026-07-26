#!/usr/bin/env python3
"""
mycobot_bridge.py -- persistent process bridging the mycobot_hardware
ros2_control plugin (mycobot_system.cpp) to the physical mycobot 280 Pi via
pymycobot, elephantrobotics' vendor driver library.

WHY A SEPARATE PYTHON PROCESS: pymycobot is Python-only (no C++ API), and
reimplementing its serial protocol in C++ from scratch would duplicate
already-tested vendor logic and be slower to get right. Instead, this
process owns the actual serial connection and exposes a tiny Unix domain
socket protocol that mycobot_system.cpp's read()/write() calls speak to.

PROTOCOL (newline-delimited JSON, one client connection at a time):
  request:  {"cmd": "read"}
  reply:    {"positions": [...7 floats, radians...],
             "velocities": [...7 floats, always 0.0 -- see NOTE below...],
             "efforts": [...7 floats, always 0.0 -- see NOTE below...]}

  request:  {"cmd": "write", "positions": [...7 floats, radians...]}
  reply:    {"ok": true}

Joint order (must match firefighter.ros2_control.xacro exactly):
  joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4,
  joint6_to_joint5, joint6output_to_joint6, gripper_controller

ARCHITECTURE -- background serial thread, non-blocking socket handler:
  Real serial round-trips to the arm (get_angles/get_gripper_value/
  send_angles/set_gripper_value) occasionally take 500-1500ms on this
  hardware (confirmed via TIMING logging on 2026-07-25/26), against a
  10ms budget from ros2_control_node's 100Hz read()/write() loop. The
  original design ran serial I/O directly inside the per-request socket
  handler, one request at a time -- a single slow call stalled every
  request behind it, and since mycobot_system.cpp times out and moves on
  after 200ms rather than waiting, the backlog grew unbounded (observed
  growing to 5+ seconds of queued_before_dispatch lag within ~10 seconds
  of runtime). That produced exactly the symptom seen on real hardware:
  commands executing many seconds after ros2_control already reported the
  goal as complete.

  Fix: a single background thread owns the arm exclusively and loops
  continuously (read state, write latest pending command if changed),
  completely decoupled from the socket. The socket handler thread(s)
  never touch the arm -- they only read/write a small shared state
  object under a lock and reply immediately. This means read() always
  returns the most recent state the background thread managed to fetch
  (which may be up to one background-loop-iteration stale, but that
  iteration proceeds at whatever pace the serial link can actually
  sustain, unblocked by anything else) and write() always records the
  latest desired command in O(1), never blocking on send_angles().

KNOWN GAPS -- confirmed against pymycobot's documented API, NOT yet verified
against the physical hardware in this project:
  - velocities are always reported as 0.0: pymycobot exposes no joint
    velocity reading. Controllers here only rely on position tracking, so
    this is a placeholder, not a bug -- but flag it if that ever changes.
  - gripper effort is always reported as 0.0: pymycobot's gripper API
    (set_gripper_value/get_gripper_value, 0-100 scale) exposes NO force/
    effort reading at all. pick_place.py's gripper_close_until_contact()
    relies on effort-based contact detection, which therefore CANNOT work
    against real hardware as written. A real fix needs a different signal
    -- e.g. is_gripper_moving() going to 0 mid-close as a stall/contact
    proxy -- and is a separate, not-yet-done piece of work.
  - DEFAULT_SERIAL_PORT/DEFAULT_BAUD_RATE below are unverified guesses for
    this exact mycobot 280 Pi unit; confirm against the actual onboard UART
    device before trusting this on real hardware.
"""

import argparse
import inspect
import json
import math
import os
import socket
import sys
import threading
import time

JOINT_ORDER = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
    "gripper_controller",
]

# Must match pick_place.py's GRIPPER_OPEN/GRIPPER_CLOSED (radians) -- kept in
# sync by hand; these two files can't share an import across packages
# without adding a dependency neither currently needs.
GRIPPER_OPEN_RAD = 0.15
GRIPPER_CLOSED_RAD = -0.60

DEFAULT_SOCKET_PATH = "/tmp/mycobot_hardware_bridge.sock"
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"  # UNVERIFIED -- confirm on real hardware
DEFAULT_BAUD_RATE = 1000000
DEFAULT_SPEED = 50  # 0-100, pymycobot's joint/gripper move speed -- see
                    # SPEED_100_RAD_PER_SEC below; this is now an UPPER BOUND,
                    # not the speed actually used for every command.

# Roughly what joint speed pymycobot's speed=100 corresponds to on this arm,
# in rad/s. UNVERIFIED against this exact unit -- measure it (command a known
# joint delta at speed 100, time it against /joint_states) and correct this if
# the speed matching below over- or under-shoots. Only the ratio matters, so an
# error here shows up as the arm consistently lagging or overrunning the
# commanded trajectory rate, not as a hard failure.
SPEED_100_RAD_PER_SEC = 2.0

# Never command below this: pymycobot speeds in the low single digits stall
# against static friction on this arm instead of moving slowly. Raised 10 -> 25
# on 2026-07-26 after runs stopped ~0.09 rad short of target: _match_speed
# scales speed to the setpoint rate, and at the END of a trajectory the
# setpoints barely move, so the last command -- the one that actually has to
# seat the arm on its target -- was going out at the floor value. Observed
# directly in a send_angles trace ending
# "...speed=21 speed=44 speed=17 speed=10", with the arm then sitting 5 degrees
# short. 10 is below what this arm needs to break static friction.
MIN_SPEED = 25

# Closed-loop settle. joint_trajectory_controller holds its final target
# forever once a trajectory elapses, so the command stops changing and
# write_command() stops marking it dirty -- meaning the bridge never sends
# anything again and the arm simply stays wherever its last open-loop
# send_angles() left it. If that was short of target (see MIN_SPEED above),
# nothing corrects it and the goal fails on the /joint_states tolerance check.
#
# These re-send the held command at FULL speed while the arm is measurably
# short of it, which is the only closed-loop position correction anywhere in
# this pipeline.
# Raised 0.02 -> 0.03 on 2026-07-26. At 0.02 a real run ended 0.0270 rad
# (1.5 deg) short and burned all 20 re-sends with the reported error not
# changing by a single digit across any of them -- twenty full-speed
# send_angles() to the same target moved the arm zero. That is servo deadband,
# not a command that failed to arrive, so 0.02 was simply below what this
# hardware can resolve and the whole budget was spent achieving nothing (10s
# of wall clock plus a burst of serial traffic at the end of EVERY move).
# Still comfortably inside pick_place.py's 0.05 convergence check.
SETTLE_TOLERANCE_RAD = 0.03
SETTLE_RESEND_INTERVAL_SEC = 0.5  # rate limit; a settle move needs time to run
SETTLE_MAX_RESENDS = 20          # ~10s, then give up rather than drive a
                                 # physically blocked joint indefinitely

# How long the commanded position must have been UNCHANGED before settling is
# allowed to fire at all.
#
# Added 2026-07-26 after the first version of the settle logic was caught
# fighting the trajectory it was supposed to be helping. Its only gate was
# "command is not dirty", which is also true in the 500-580ms dead time
# between two setpoints mid-trajectory -- so it fired while the arm was
# legitimately in transit, reported nonsense like "still 0.5129 rad short"
# (of course it is, it is halfway there), and re-commanded at FULL speed while
# the trajectory was deliberately pacing at speed 25. Roughly 7 such spurious
# full-speed darts were injected into a single 2.6s homing move, close to
# doubling the number of conflicting commands and making the jerk worse.
#
# The real signal for "the trajectory is over" is that the commanded position
# has stopped CHANGING: joint_trajectory_controller streams a fresh setpoint
# every control cycle while a trajectory runs, then holds one constant target
# forever once it elapses. Anything above the observed inter-setpoint gap
# (~0.6s worst case) works; 1.0s leaves margin without adding meaningful
# latency, since settling only ever matters after the motion has stopped.
SETTLE_QUIET_PERIOD_SEC = 1.0

# Give up early when settling is not achieving anything: if the position error
# improves by less than this between consecutive re-sends, count it as a stall.
SETTLE_MIN_PROGRESS_RAD = 0.002
SETTLE_MAX_STALLED = 3

# ASYNC WRITES -- the fix for the 1.8Hz command rate (2026-07-26).
#
# pymycobot's send_angles() defaults to has_reply=True, i.e. it BLOCKS until
# the arm's firmware acknowledges. Reading pymycobot 4.0.6's source on the
# robot showed exactly what that costs:
#
#   common.py read():   wait_time = 0.15 on Windows, else 0.5   <-- Linux
#                       while True and time.time() - t < wait_time: ...
#   mycobot280.py _res(): retries the whole thing up to 3 times
#
# So a call whose reply never arrives burns 0.5s, and one that fails outright
# burns 3 x 0.5s and returns -1. Every number in the 2026-07-26 robot log is a
# direct prediction of that code: writes were bimodal at ~2-5ms (firmware
# replied) or ~510-590ms (one timeout, then a retry that worked); one read
# took 1506ms (all three attempts timed out); and get_angles() returned -1 in
# the same window (the `else: return -1` after three failures). Nothing about
# it was flaky hardware.
#
# Consequence: the serial loop ran at 0.9-1.9Hz WHILE THE ARM WAS MOVING (and
# 82-89Hz idle), so a 2.6s trajectory reached the arm as ~11 point-to-point
# commands, one of which asked the base joint to jump 31 degrees. That is the
# jerky motion, and it is self-inflicted -- the blocking only happens because
# the arm is busy executing the move we just sent it.
#
# _mesg()'s _async branch is a pure self._write() with no read, no timeout and
# no retry: ~0.2ms for a 16-byte frame at 1Mbaud. It returns None instead of a
# status, which costs nothing here since the return value was never used. The
# firmware's deferred replies do still land in the input buffer, but _res()
# calls reset_input_buffer() before every read, so the next get_angles()
# flushes them automatically.
#
# Kept switchable (--sync-writes) because this is a behavioural change against
# real hardware and being able to A/B it in one run is worth the flag.
DEFAULT_ASYNC_WRITES = True

# Ceiling on how often a position command is handed to the arm.
#
# With async writes the serial loop is no longer throttled by the write at all
# -- it runs at whatever get_angles() allows (~85Hz), and every iteration would
# otherwise fire a fresh send_angles(). Each one still ABORTS AND RESTARTS the
# move in progress, and re-commanding a servo ~85 times a second is a good way
# to get buzzing instead of motion. 30Hz is ~16x the old effective rate, which
# turns those 31-degree jumps into sub-degree steps (i.e. into something that
# approximates the servo interface joint_trajectory_controller thinks it is
# talking to), while still leaving each command real time to take effect.
#
# UNVERIFIED as an optimum -- it is a starting point, not a measured value.
# Sweep it with --max-command-rate and watch for the arm lagging the
# trajectory (too low) or vibrating/buzzing (too high).
DEFAULT_MAX_COMMAND_RATE_HZ = 30.0

# Minimum spacing between arm state reads WHILE THE SETPOINT IS MOVING. Reads
# run at full rate (every loop iteration) whenever it is not. See
# _serial_read_once_if_due() for the measurements behind this.
#
# Raised 0.5 -> 1.5 on 2026-07-27. At 0.5 the dt= column showed the cost
# directly: a steady stream of dt=34-36ms commands interrupted every half
# second by a single dt=551ms one, across which a joint target jumped 11
# degrees and another 15.6. Nothing can be written while a read holds the
# serial link, JTC's trajectory clock keeps running through the blackout, and
# the next command to land is wherever the trajectory has got to by then -- so
# the arm stops and then lurches. A 5s move at a 0.5s interval gives about
# five of those, which is exactly the "4 or 5 jerks per move" reported from
# the hardware.
#
# This only makes the blackouts RARER. See DEFAULT_READ_TIMEOUT_SEC for the
# other half, which makes each one shorter.
DEFAULT_MOTION_READ_INTERVAL_SEC = 1.5

# Cap on how long a single pymycobot read may block, in seconds.
#
# pymycobot's common.py read() hardcodes wait_time=0.5 on Linux and its
# caller _res() retries three times, so one get_angles() can hold the serial
# link for 1.5s -- observed as `loop_rate=1.3Hz (last read 1507ms)` followed
# by `get_angles() returned -1`. While it is blocked, no position command can
# be written, which is what the arm sees as a jerk.
#
# read() does accept a `timeout` argument that overrides wait_time, but
# nothing in the call chain from get_angles() passes one, so the only way to
# supply it is to wrap _read (see _install_read_timeout). The arm answers a
# healthy read in 10-25ms, so 0.1s is generous for a good reply while cutting
# a bad one from 500ms to 100ms and a fully failed one from 1.5s to 0.3s.
#
# The cost is that a genuinely slow-but-valid reply now gets abandoned, making
# the reported state staler. That is the right trade here: nothing closes a
# loop on measured position during a trajectory, and _serial_read_once already
# keeps the last known values when a read returns -1.
DEFAULT_READ_TIMEOUT_SEC = 0.1

# Position command must change by at least this much (radians) before the
# background loop bothers re-sending it to the arm -- avoids spamming
# send_angles()/set_gripper_value() with the same target every loop
# iteration, which just adds pointless serial traffic and further starves
# the read() side of the loop.
COMMAND_CHANGE_EPSILON_RAD = 0.001


def gripper_rad_to_value(rad):
    """Map GRIPPER_CLOSED_RAD..GRIPPER_OPEN_RAD onto pymycobot's
    0 (closed) .. 100 (open) gripper_value scale."""
    span = GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD
    frac = (rad - GRIPPER_CLOSED_RAD) / span
    frac = max(0.0, min(1.0, frac))
    return int(round(frac * 100))


def gripper_value_to_rad(value):
    span = GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD
    frac = max(0.0, min(100.0, value)) / 100.0
    return GRIPPER_CLOSED_RAD + frac * span


class SharedState:
    """State shared between the background serial thread and the socket
    handler thread(s), guarded by a single lock. Kept intentionally tiny --
    every access is O(1) and non-blocking, since the socket handler must
    never wait on this lock for long."""

    def __init__(self, n_joints):
        self.lock = threading.Lock()
        self.positions = [0.0] * n_joints
        self.velocities = [0.0] * n_joints
        self.efforts = [0.0] * n_joints
        self.last_read_monotonic = 0.0
        self.command = None          # latest requested positions, or None
        self.command_dirty = False   # True until the background loop has sent it
        # Last command actually handed to the arm over serial, as opposed to
        # `command` which is only the last one RECEIVED over the socket. Kept
        # separate so a write that fails or is skipped can be retried -- see
        # _serial_write_once_if_dirty()'s comment on the lost-command latch.
        self.last_sent = None
        self.last_sent_monotonic = 0.0
        # Closed-loop settle bookkeeping -- see _serial_settle_if_needed().
        self.last_settle_monotonic = 0.0
        self.settle_resends = 0
        # When the commanded position last actually CHANGED (as opposed to
        # being re-received unchanged every control cycle). This is what tells
        # settling that the trajectory is over rather than merely between
        # setpoints -- see SETTLE_QUIET_PERIOD_SEC.
        self.command_changed_monotonic = 0.0
        # Error at the previous settle attempt, for stall detection.
        self.settle_last_error = None
        self.settle_stalled = 0


class Bridge:
    def __init__(self, serial_port, baud_rate, speed, log_timing=True,
                 async_writes=True, max_command_rate_hz=DEFAULT_MAX_COMMAND_RATE_HZ,
                 motion_read_interval=DEFAULT_MOTION_READ_INTERVAL_SEC,
                 min_speed=MIN_SPEED,
                 read_timeout=DEFAULT_READ_TIMEOUT_SEC):
        from pymycobot import MyCobot280  # imported here so --help works without hardware attached

        self.speed = speed
        self.log_timing = log_timing
        self.async_writes = async_writes
        self.min_command_period = (1.0 / max_command_rate_hz
                                   if max_command_rate_hz > 0 else 0.0)
        self.motion_read_interval = motion_read_interval
        self.min_speed = min_speed
        print(f"[mycobot_bridge] connecting to {serial_port} @ {baud_rate}...")
        self.arm = MyCobot280(serial_port, baud_rate)
        print("[mycobot_bridge] connected.")
        self._install_read_timeout(read_timeout)
        self.state = SharedState(len(JOINT_ORDER))
        self._stop = threading.Event()
        self._read_count = 0

    def _install_read_timeout(self, timeout_sec):
        """Wrap pymycobot's _read so every read carries an explicit timeout.

        read() already supports one -- `if timeout is not None: wait_time =
        timeout` -- but no call path from get_angles()/get_gripper_value()
        supplies it, so the hardcoded 0.5s Linux default always wins. Wrapping
        the bound method is the least invasive way to inject it: no vendor file
        is edited, and if a future pymycobot drops the parameter this detects
        that and leaves the default in place rather than raising."""
        if not timeout_sec or timeout_sec <= 0:
            print("[mycobot_bridge] read timeout: using pymycobot's default "
                  "(0.5s per attempt, 3 attempts)")
            return

        original_read = self.arm._read
        try:
            parameters = inspect.signature(original_read).parameters
        except (TypeError, ValueError):
            parameters = {}
        if "timeout" not in parameters:
            print("[mycobot_bridge] WARNING: this pymycobot's _read takes no "
                  "'timeout' argument -- leaving its default in place. Reads "
                  "may block up to 1.5s and cause visible jerks.")
            return

        def read_with_timeout(genre, *args, **kwargs):
            kwargs.setdefault("timeout", timeout_sec)
            return original_read(genre, *args, **kwargs)

        self.arm._read = read_with_timeout
        print(f"[mycobot_bridge] read timeout capped at {timeout_sec}s per "
              f"attempt (pymycobot's Linux default is 0.5s)")

    # ---- background thread: owns the arm exclusively ----

    def serial_loop(self):
        """Runs continuously on its own thread for the lifetime of the
        process. Never touched by the socket handler -- this is the only
        code that calls into pymycobot, so a slow/stalled serial call here
        blocks nothing except this loop's own next iteration.

        The loop's ACTUAL iteration rate is the single most important unknown
        in this whole pipeline and is reported once a second when log_timing
        is on. It bounds how many distinct position commands the arm can
        possibly receive: ros2_control_node's joint_trajectory_controller
        interpolates setpoints at 100Hz, but the arm only ever sees the
        latest one per iteration of THIS loop. If this loop runs at ~1Hz (two
        serial reads plus a write per iteration, at the 500-1500ms round
        trips measured on 2026-07-25/26), then a 9-second trajectory reaches
        the arm as roughly 9 point-to-point send_angles() commands rather
        than 900 servo setpoints -- and since each send_angles() ABORTS AND
        RESTARTS the previous move at self.speed, the arm spends the whole
        trajectory darting toward a stale setpoint and stopping. Compare that
        number against the trajectory's waypoint count and duration (now
        printed by pick_place.py's _describe_trajectory) before blaming
        DDS."""
        print("[mycobot_bridge] serial loop starting")
        iterations = 0
        last_read_ms = 0.0
        window_start = time.monotonic()
        while not self._stop.is_set():
            t0 = time.monotonic()
            did_read = self._serial_read_once_if_due()
            if did_read:
                last_read_ms = 1000 * (time.monotonic() - t0)
            self._serial_write_once_if_dirty()
            self._serial_settle_if_needed()
            iterations += 1

            if not did_read:
                # Nothing here blocks when the read is skipped and the write is
                # rate limited, so without this the loop spins on the CPU for
                # no benefit. 2ms is far finer than the write cap's period.
                time.sleep(0.002)

            if self.log_timing:
                elapsed = time.monotonic() - window_start
                if elapsed >= 1.0:
                    print(f"[mycobot_bridge] TIMING loop_rate={iterations / elapsed:.1f}Hz "
                          f"(last read {last_read_ms:.0f}ms) -- this is the "
                          f"real ceiling on commands/sec reaching the arm")
                    iterations = 0
                    window_start = time.monotonic()

    def _serial_read_once_if_due(self):
        """Read arm state, but not on every iteration while the arm is moving.

        WHY (measured 2026-07-27, after async writes fixed the write side):
        get_angles() costs 10-25ms while the arm is parked and 500-2000ms while
        it is moving -- the firmware does not answer promptly when it is busy
        executing a move. That is the same 0.5s timeout plus 3x retry in
        pymycobot's read()/_res() that made writes slow, and a 2021ms read in
        that run was immediately preceded by `get_angles() returned -1`, i.e.
        all three attempts timing out. It is NOT caused by async writes leaving
        unacknowledged replies in the buffer: reads were already 535-1506ms
        during motion in the pre-async logs.

        Since the write now costs ~0ms, the read is the entire loop period, so
        a 4.4s trajectory still reached the arm as only 11 commands -- one of
        which asked a joint to move 58 degrees in a single point-to-point move.

        Nothing closes a feedback loop on the measured position DURING a
        trajectory (joint_trajectory_controller is time-based here, and the
        settle logic deliberately waits for motion to stop), so a stale reading
        mid-move costs little, whereas a stale COMMAND costs tracking accuracy
        directly. Reads therefore fall back to a slow poll while the setpoint
        is moving and return to full rate the moment it stops -- which is
        before the next goal starts, so a new trajectory still begins from
        fresh state.

        Returns True if a read was actually performed."""
        now = time.monotonic()
        with self.state.lock:
            moving = (
                self.state.command_changed_monotonic > 0.0
                and now - self.state.command_changed_monotonic < SETTLE_QUIET_PERIOD_SEC
            )
            since_read = now - self.state.last_read_monotonic

        if (moving and self.motion_read_interval > 0.0
                and since_read < self.motion_read_interval):
            return False

        self._serial_read_once()
        return True

    # The gripper position is read once every this many loop iterations
    # instead of every one. get_angles() and get_gripper_value() are two
    # separate serial round trips at 500-1500ms each, so reading both every
    # iteration doubles the loop period -- and the loop period is the hard
    # ceiling on how many position commands per second can reach the arm (see
    # serial_loop's docstring). The gripper is only ever commanded to a small
    # number of discrete positions and nothing closes a feedback loop on its
    # reported value (pymycobot exposes no gripper effort at all -- see the
    # module docstring), so a stale gripper reading costs nothing, whereas a
    # halved arm command rate costs real tracking accuracy.
    GRIPPER_READ_EVERY = 10

    def _serial_read_once(self):
        # pymycobot's get_* calls commonly return -1 (a truthy int, not
        # None/0/[]) on a communication error/timeout instead of raising --
        # observed for real over serial: `get_angles()` returned a bare int,
        # which crashed here (and took the whole bridge process down with
        # it, since nothing caught it) when iterated as if it were the
        # expected 6-element list. Validate shape explicitly rather than
        # relying on truthiness.
        try:
            angles_deg = self.arm.get_angles()
            if not isinstance(angles_deg, (list, tuple)) or len(angles_deg) != 6:
                print(f"[mycobot_bridge] WARNING: get_angles() returned "
                      f"{angles_deg!r}, expected a 6-element list -- keeping "
                      f"last known positions for this read")
                angles_deg = None

            self._read_count += 1
            # == 1, not == 0, so the very first read (the seeding read in
            # main(), before any client connects) always includes the gripper
            # rather than leaving it at its 0.0 placeholder for 10 iterations.
            if self._read_count % self.GRIPPER_READ_EVERY == 1:
                gripper_value = self.arm.get_gripper_value()
                if not isinstance(gripper_value, (int, float)):
                    print(f"[mycobot_bridge] WARNING: get_gripper_value() returned "
                          f"{gripper_value!r}, expected a number -- keeping last "
                          f"known gripper position for this read")
                    gripper_value = None
            else:
                gripper_value = None  # keep the last known value, see GRIPPER_READ_EVERY
        except Exception as exc:
            # A transient serial exception must not kill the background
            # thread (and with it, forever, all future reads/writes) --
            # log it and try again next iteration.
            print(f"[mycobot_bridge] ERROR during serial read: {exc!r}")
            return

        with self.state.lock:
            if angles_deg is not None:
                for i, a in enumerate(angles_deg):
                    self.state.positions[i] = math.radians(a)
            if gripper_value is not None:
                self.state.positions[6] = gripper_value_to_rad(gripper_value)
            self.state.last_read_monotonic = time.monotonic()

    def _send_angles(self, arm_degrees, speed):
        """send_angles(), async by default -- see DEFAULT_ASYNC_WRITES for why
        the synchronous form costs 0.5-1.5s per call on Linux."""
        if self.async_writes:
            self.arm.send_angles(arm_degrees, speed, _async=True)
        else:
            self.arm.send_angles(arm_degrees, speed)

    def _serial_write_once_if_dirty(self):
        with self.state.lock:
            if not self.state.command_dirty or self.state.command is None:
                return
            # Rate limit. Returning WITHOUT clearing command_dirty is what
            # makes the next loop iteration pick this up again -- and since
            # `command` is re-read then, a newer setpoint supersedes this one
            # rather than queueing behind it, which is the correct behaviour
            # for a servo stream.
            if (self.min_command_period > 0.0
                    and time.monotonic() - self.state.last_sent_monotonic
                    < self.min_command_period):
                return
            positions = list(self.state.command)
            last_sent = self.state.last_sent
            last_sent_at = self.state.last_sent_monotonic

        arm_degrees = [math.degrees(p) for p in positions[:6]]
        gripper_value = gripper_rad_to_value(positions[6])
        speed = self._match_speed(positions, last_sent, last_sent_at)

        # Only touch the gripper when the GRIPPER command actually changed.
        # Previously set_gripper_value() was re-sent on every arm command
        # change too, which during a multi-second arm trajectory means one
        # redundant serial round trip per control cycle, all of it competing
        # with the arm's own send_angles() on the same 1 Mbaud UART for no
        # benefit -- the gripper target hadn't moved.
        gripper_changed = (
            last_sent is None
            or abs(positions[6] - last_sent[6]) > COMMAND_CHANGE_EPSILON_RAD
        )

        t0 = time.monotonic()
        try:
            self._send_angles(arm_degrees, speed)
            t_arm = time.monotonic()
            if gripper_changed:
                self.arm.set_gripper_value(gripper_value, self.speed)
            t_grip = time.monotonic()
        except Exception as exc:
            # Do NOT clear command_dirty here: leaving it set is what makes
            # the next loop iteration retry this command.
            print(f"[mycobot_bridge] ERROR during serial write: {exc!r} "
                  f"-- leaving command dirty so it gets retried")
            return

        with self.state.lock:
            self.state.last_sent = positions
            self.state.last_sent_monotonic = t0
            # A new commanded position restarts the settle budget.
            self.state.settle_resends = 0
            # Clear the dirty flag only if no NEWER command arrived while we
            # were busy on the serial link (which takes 500-1500ms here, i.e.
            # 50-150 control cycles).
            #
            # THE LOST-COMMAND LATCH (fixed 2026-07-26): this flag used to be
            # cleared up front, before the write. If the write then failed --
            # or was simply skipped -- that command was silently dropped, and
            # because write_command() compares each incoming socket command
            # against `state.command` (which had ALREADY been updated to this
            # value), no subsequent identical command could ever re-dirty it.
            # That is terminal at the END of a trajectory specifically:
            # joint_trajectory_controller holds its final target constant
            # forever once the trajectory elapses, so every later write() is
            # byte-identical, `changed` is False every time, and the arm never
            # receives the final target at all -- while the controller still
            # reports "Goal reached, success!" from elapsed time alone. Exactly
            # the observed "goal accepted, success logged, arm never moved".
            if self.state.command == positions:
                self.state.command_dirty = False

        if self.log_timing:
            gripper_note = "(sent)" if gripper_changed else "(skipped)"
            # dt is the gap since the PREVIOUS command reached the arm, and is
            # the single most useful number here for diagnosing visible skips:
            # a stutter in an otherwise smooth motion is a command blackout, and
            # a blackout shows up as one dt of hundreds/thousands of ms in a
            # stream of ~33ms ones. The likeliest cause is a get_angles() that
            # hit all three of pymycobot's 0.5s retries (2021ms observed), since
            # nothing can be written while the serial link is blocked on it.
            dt_note = ("dt=first" if last_sent_at <= 0.0
                       else f"dt={1000 * (t0 - last_sent_at):.0f}ms")
            print(f"[mycobot_bridge] TIMING {dt_note} "
                  f"send_angles={1000 * (t_arm - t0):.0f}ms "
                  f"gripper={1000 * (t_grip - t_arm):.0f}ms {gripper_note} "
                  f"speed={speed}  "
                  f"target_deg={[round(d, 2) for d in arm_degrees]}")

    def _serial_settle_if_needed(self):
        """Re-send the currently-held command at full speed while the arm is
        measurably short of it.

        This is the only closed-loop position correction in the pipeline.
        Everything else here is open loop: joint_trajectory_controller streams
        setpoints, the bridge forwards the newest one as a point-to-point
        send_angles(), and once the trajectory elapses JTC holds its final
        target forever. A held target never changes, so write_command() stops
        marking it dirty and the bridge falls silent -- leaving the arm
        wherever its last send_angles() happened to stop.

        That is exactly the 2026-07-26 intermittent failure: runs travelled
        ~1.72 rad and settled 0.09 rad (5 deg) short of target, failing
        pick_place.py's 0.05 rad convergence check, with no further command
        ever issued to close the gap. Re-sending at full speed corrects it.

        ONLY ONCE THE TRAJECTORY IS OVER. "Not dirty" is NOT sufficient to
        establish that -- it is equally true during the 500-580ms gap between
        two mid-trajectory setpoints, and the first version of this function
        fired there, correcting an arm that was simply still in transit and
        doing it at full speed against a trajectory pacing at speed 25. The
        actual end-of-trajectory signal is the commanded position having
        stopped changing for SETTLE_QUIET_PERIOD_SEC.

        ARM JOINTS ONLY (positions[:6]). The gripper is deliberately excluded:
        when it is holding a block it CANNOT reach its commanded value, and
        that is the success condition, not an error -- settling it would just
        drive the jaw harder into the object forever."""
        now = time.monotonic()
        with self.state.lock:
            if self.state.command is None or self.state.command_dirty:
                return
            # The trajectory is still running if the setpoint moved recently.
            if now - self.state.command_changed_monotonic < SETTLE_QUIET_PERIOD_SEC:
                return
            if now - self.state.last_settle_monotonic < SETTLE_RESEND_INTERVAL_SEC:
                return
            if self.state.settle_resends >= SETTLE_MAX_RESENDS:
                return
            if self.state.settle_stalled >= SETTLE_MAX_STALLED:
                return
            command = list(self.state.command)
            measured = list(self.state.positions)
            attempt = self.state.settle_resends + 1
            previous_error = self.state.settle_last_error

        error = max(abs(c - p) for c, p in zip(command[:6], measured[:6]))
        if error <= SETTLE_TOLERANCE_RAD:
            return

        # Re-sending the same target to an arm that is not responding to it
        # just burns the budget and floods the serial link. Observed for real:
        # 20 consecutive re-sends against a 0.0270 rad error that did not move
        # by a single digit. Stop after a few no-progress attempts instead.
        stalled = (previous_error is not None
                   and previous_error - error < SETTLE_MIN_PROGRESS_RAD)

        arm_degrees = [math.degrees(p) for p in command[:6]]
        try:
            self._send_angles(arm_degrees, self.speed)
        except Exception as exc:
            print(f"[mycobot_bridge] ERROR during settle re-send: {exc!r}")
            return

        with self.state.lock:
            self.state.last_settle_monotonic = now
            self.state.settle_resends = attempt
            self.state.settle_last_error = error
            self.state.settle_stalled = (
                self.state.settle_stalled + 1 if stalled else 0)
            gave_up = self.state.settle_stalled >= SETTLE_MAX_STALLED

        if self.log_timing:
            print(f"[mycobot_bridge] TIMING settle re-send {attempt}/"
                  f"{SETTLE_MAX_RESENDS}: still {error:.4f} rad short "
                  f"(> {SETTLE_TOLERANCE_RAD}), re-commanding at speed {self.speed}")
            if gave_up:
                print(f"[mycobot_bridge] TIMING settle giving up: {error:.4f} rad "
                      f"error stopped improving over {SETTLE_MAX_STALLED} attempts "
                      f"-- most likely servo deadband or a physically blocked joint, "
                      f"not a lost command")

    def _match_speed(self, positions, last_sent, last_sent_at):
        """Pick the pymycobot speed that makes the arm arrive at `positions`
        just as the NEXT command is due, instead of always darting there at
        self.speed and then sitting idle.

        WHY (root cause of the 2026-07-26 "goal accepted, arm never moved"
        failure, diagnosed by measurement rather than inference):

        joint_trajectory_controller is a servo interface -- it interpolates
        the planned path into a fresh position setpoint every control cycle
        (100Hz) and expects the hardware to track it. pymycobot's
        send_angles(angles, speed) is the opposite thing: a point-to-point
        MOVE command that the arm's own firmware executes asynchronously over
        hundreds of milliseconds, and which ABORTS AND RESTARTS whatever move
        was already in progress.

        This loop can only forward the newest setpoint once per iteration, and
        an iteration costs a full serial round trip (500-1500ms measured). So
        a 9.14s planned trajectory -- 920 setpoints at 100Hz -- reaches the arm
        as roughly 5-20 send_angles() calls. At a fixed speed=50 (~1.0 rad/s,
        i.e. 5x faster than the 0.2 rad/s the trajectory actually asks for)
        each call darts ~0.1-0.2 rad ahead in ~0.1s and then the arm sits
        still for the remaining ~0.9s until the next one lands. Net motion
        across the whole trajectory is a stutter that covers a fraction of the
        distance -- and because ros2_controllers.yaml declares no
        `constraints:` block for arm_group_controller, the controller has no
        goal tolerance to check and reports "Goal reached, success!" purely
        from elapsed trajectory time regardless.

        Single-waypoint trajectories (joint_trajectory_test.py, and every
        motion this project has ever confirmed on real hardware) escape this
        entirely: once the trajectory's duration elapses the controller holds
        ONE constant target forever, so exactly one send_angles() runs to
        completion uninterrupted and the firmware drives the whole way there.
        The motion that has been observed working was always that
        post-trajectory hold, never trajectory tracking.

        Matching the speed to the actual setpoint rate turns the stutter into
        continuous motion: each command is still aborted and replaced, but it
        is replaced at roughly the point the arm has just reached, moving at
        roughly the right velocity, so the aborts stop mattering. Returns
        self.speed unchanged when there's no history to estimate from (the
        first write after activation, where a point-to-point dart is
        exactly what's wanted)."""
        if last_sent is None or last_sent_at <= 0.0:
            return self.speed

        dt = time.monotonic() - last_sent_at
        if dt <= 0.0:
            return self.speed

        max_delta = max(abs(a - b) for a, b in zip(positions[:6], last_sent[:6]))
        if max_delta <= COMMAND_CHANGE_EPSILON_RAD:
            # Setpoint has effectively stopped moving -- this is the
            # post-trajectory hold. Use the full configured speed so the arm
            # closes out the last of the distance promptly.
            return self.speed

        required_rad_s = max_delta / dt
        speed = int(round(100.0 * required_rad_s / SPEED_100_RAD_PER_SEC))
        # Never exceed the configured ceiling and never stall below min_speed.
        return max(self.min_speed, min(self.speed, speed))

    # ---- socket handler thread(s): never touch the arm directly ----

    def read_state(self):
        with self.state.lock:
            return {
                "positions": list(self.state.positions),
                "velocities": list(self.state.velocities),  # always 0.0, see module docstring
                "efforts": list(self.state.efforts),          # always 0.0, see module docstring
            }

    def write_command(self, positions):
        if len(positions) != len(JOINT_ORDER):
            print(f"[mycobot_bridge] WARNING: expected {len(JOINT_ORDER)} "
                  f"positions, got {len(positions)} -- ignoring write")
            return

        with self.state.lock:
            changed = (
                self.state.command is None
                or any(
                    abs(a - b) > COMMAND_CHANGE_EPSILON_RAD
                    for a, b in zip(positions, self.state.command)
                )
            )
            self.state.command = list(positions)
            if changed:
                self.state.command_dirty = True
                # Stamped here, on the CHANGE, not on every write: JTC calls
                # write() every control cycle whether or not the setpoint
                # moved, so "time since last write" says nothing, while "time
                # since the setpoint last moved" is exactly the
                # end-of-trajectory signal settling needs.
                self.state.command_changed_monotonic = time.monotonic()
                # A new target invalidates the previous settle attempt's error.
                self.state.settle_last_error = None
                self.state.settle_stalled = 0

    def handle_client(self, conn):
        buf = b""
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self._handle_line(conn, line)

    def _handle_line(self, conn, line):
        try:
            request = json.loads(line)
        except json.JSONDecodeError as exc:
            print(f"[mycobot_bridge] bad request, ignoring: {exc}")
            return

        cmd = request.get("cmd")
        if cmd == "read":
            reply = self.read_state()
        elif cmd == "write":
            self.write_command(request.get("positions", []))
            reply = {"ok": True}
        else:
            reply = {"error": f"unknown cmd {cmd!r}"}

        # Echo the request's id back so mycobot_system.cpp can tell this
        # reply apart from a stale one left over from a request it already
        # gave up waiting on -- see send_request()'s comment in
        # mycobot_system.cpp for why that matters.
        reply["id"] = request.get("id")

        conn.sendall((json.dumps(reply) + "\n").encode())

    def stop(self):
        self._stop.set()


def main():
    # Line-buffer stdout. Python block-buffers (~4-8kB) whenever stdout is not
    # a TTY, which is exactly the case under `ros2 launch` -- so every print
    # here, including the startup banner and all the TIMING lines, arrives in
    # delayed bursts instead of in order. That is not merely cosmetic: it makes
    # bridge output impossible to correlate with ros2_control_node's timestamps,
    # and on 2026-07-27 it hid the startup banner entirely for the whole of a
    # launch, so there was no way to confirm which write mode was actually
    # active. Done here rather than via PYTHONUNBUFFERED in the launch file so
    # it holds however this process gets started.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass  # Python < 3.7; the robot is on 3.8, so this is belt and braces

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baud-rate", type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="pymycobot move speed, 0-100 (default %(default)s)")
    parser.add_argument("--sync-writes", dest="async_writes", action="store_false",
                        help="send position commands with pymycobot's default "
                             "blocking send_angles() instead of the _async=True "
                             "form. Restores the pre-2026-07-26 behaviour, where "
                             "every write could cost 0.5s (one read timeout) or "
                             "1.5s (three) and the serial loop ran at 1-2Hz while "
                             "the arm moved. Here to A/B the change against real "
                             "hardware, not because it is a good idea.")
    parser.add_argument("--max-command-rate", type=float,
                        default=DEFAULT_MAX_COMMAND_RATE_HZ,
                        help="ceiling on position commands/sec handed to the arm "
                             "(default %(default)s, 0 disables). Each command "
                             "aborts and restarts the move in progress, so this "
                             "trades tracking accuracy against re-commanding the "
                             "servos so often they buzz instead of moving.")
    parser.add_argument("--read-timeout", type=float,
                        default=DEFAULT_READ_TIMEOUT_SEC,
                        help="cap on how long one pymycobot read may block, in "
                             "seconds (default %(default)s, 0 keeps pymycobot's "
                             "hardcoded 0.5s per attempt with 3 attempts). No "
                             "position command can be written while a read holds "
                             "the serial link, so this bounds the command "
                             "blackout that the arm feels as a jerk.")
    parser.add_argument("--min-speed", type=int, default=MIN_SPEED,
                        help="floor for the speed matching (default %(default)s). "
                             "25 was chosen on 2026-07-26 for the OLD regime, "
                             "where commands reached the arm at ~2Hz and the "
                             "final one had to break static friction on its own. "
                             "Neither still holds: settling now re-sends at full "
                             "speed once motion stops, and at 30Hz each command "
                             "covers well under a degree, for which 25 is roughly "
                             "twice as fast as needed -- so the arm darts and "
                             "waits 30 times a second instead of moving "
                             "continuously. Try 10-15 if the motion looks buzzy.")
    parser.add_argument("--motion-read-interval", type=float,
                        default=DEFAULT_MOTION_READ_INTERVAL_SEC,
                        help="minimum seconds between arm state reads while the "
                             "setpoint is moving (default %(default)s, 0 reads "
                             "every iteration). get_angles() costs 10-25ms parked "
                             "but 500-2000ms mid-move, so reading every iteration "
                             "spends the entire loop period on state nothing acts "
                             "on until the motion stops.")
    parser.add_argument("--no-log-timing", dest="log_timing", action="store_false",
                        help="suppress the per-second serial loop_rate line and the "
                             "per-write send_angles timing/target lines. On by "
                             "default: these are the only direct evidence of "
                             "whether a commanded position ever reached the servos, "
                             "and their absence is what made the 2026-07-25/26 "
                             "'goal succeeded but the arm never moved' failures "
                             "undiagnosable.")
    args = parser.parse_args()

    bridge = Bridge(args.serial_port, args.baud_rate, args.speed,
                    log_timing=args.log_timing,
                    async_writes=args.async_writes,
                    max_command_rate_hz=args.max_command_rate,
                    motion_read_interval=args.motion_read_interval,
                    min_speed=args.min_speed,
                    read_timeout=args.read_timeout)
    print(f"[mycobot_bridge] writes={'async' if args.async_writes else 'sync (blocking)'}, "
          f"max command rate={args.max_command_rate}Hz, "
          f"motion read interval={args.motion_read_interval}s, "
          f"speed={args.min_speed}..{args.speed}")

    # Seed shared state with a real initial read before accepting any
    # connections, so the first read() a client makes doesn't race the
    # background thread's first iteration and return all-zero positions.
    bridge._serial_read_once()

    serial_thread = threading.Thread(target=bridge.serial_loop, daemon=True)
    serial_thread.start()

    if os.path.exists(args.socket_path):
        os.remove(args.socket_path)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(args.socket_path)
    server.listen(1)
    print(f"[mycobot_bridge] listening on {args.socket_path}")

    try:
        while True:
            conn, _ = server.accept()
            print("[mycobot_bridge] mycobot_system.cpp connected")
            try:
                bridge.handle_client(conn)
            finally:
                conn.close()
                print("[mycobot_bridge] client disconnected, waiting for reconnect")
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        server.close()
        if os.path.exists(args.socket_path):
            os.remove(args.socket_path)


if __name__ == "__main__":
    main()
