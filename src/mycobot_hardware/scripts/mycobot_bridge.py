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
import json
import math
import os
import socket
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
SETTLE_TOLERANCE_RAD = 0.02      # tighter than pick_place.py's 0.05 check, so
                                 # settling actually clears that threshold
SETTLE_RESEND_INTERVAL_SEC = 0.5  # rate limit; a settle move needs time to run
SETTLE_MAX_RESENDS = 20          # ~10s, then give up rather than drive a
                                 # physically blocked joint indefinitely

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


class Bridge:
    def __init__(self, serial_port, baud_rate, speed, log_timing=True):
        from pymycobot import MyCobot280  # imported here so --help works without hardware attached

        self.speed = speed
        self.log_timing = log_timing
        print(f"[mycobot_bridge] connecting to {serial_port} @ {baud_rate}...")
        self.arm = MyCobot280(serial_port, baud_rate)
        print("[mycobot_bridge] connected.")
        self.state = SharedState(len(JOINT_ORDER))
        self._stop = threading.Event()
        self._read_count = 0

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
        window_start = time.monotonic()
        while not self._stop.is_set():
            t0 = time.monotonic()
            self._serial_read_once()
            t_read = time.monotonic()
            self._serial_write_once_if_dirty()
            self._serial_settle_if_needed()
            iterations += 1

            if self.log_timing:
                elapsed = time.monotonic() - window_start
                if elapsed >= 1.0:
                    print(f"[mycobot_bridge] TIMING loop_rate={iterations / elapsed:.1f}Hz "
                          f"(last read {1000 * (t_read - t0):.0f}ms) -- this is the "
                          f"real ceiling on commands/sec reaching the arm")
                    iterations = 0
                    window_start = time.monotonic()

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

    def _serial_write_once_if_dirty(self):
        with self.state.lock:
            if not self.state.command_dirty or self.state.command is None:
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
            self.arm.send_angles(arm_degrees, speed)
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
            print(f"[mycobot_bridge] TIMING send_angles={1000 * (t_arm - t0):.0f}ms "
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

        ARM JOINTS ONLY (positions[:6]). The gripper is deliberately excluded:
        when it is holding a block it CANNOT reach its commanded value, and
        that is the success condition, not an error -- settling it would just
        drive the jaw harder into the object forever."""
        now = time.monotonic()
        with self.state.lock:
            if self.state.command is None or self.state.command_dirty:
                return
            if now - self.state.last_settle_monotonic < SETTLE_RESEND_INTERVAL_SEC:
                return
            if self.state.settle_resends >= SETTLE_MAX_RESENDS:
                return
            command = list(self.state.command)
            measured = list(self.state.positions)
            attempt = self.state.settle_resends + 1

        error = max(abs(c - p) for c, p in zip(command[:6], measured[:6]))
        if error <= SETTLE_TOLERANCE_RAD:
            return

        arm_degrees = [math.degrees(p) for p in command[:6]]
        try:
            self.arm.send_angles(arm_degrees, self.speed)
        except Exception as exc:
            print(f"[mycobot_bridge] ERROR during settle re-send: {exc!r}")
            return

        with self.state.lock:
            self.state.last_settle_monotonic = now
            self.state.settle_resends = attempt

        if self.log_timing:
            print(f"[mycobot_bridge] TIMING settle re-send {attempt}/"
                  f"{SETTLE_MAX_RESENDS}: still {error:.4f} rad short "
                  f"(> {SETTLE_TOLERANCE_RAD}), re-commanding at speed {self.speed}")

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
        # Never exceed the configured ceiling and never stall below MIN_SPEED.
        return max(MIN_SPEED, min(self.speed, speed))

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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baud-rate", type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="pymycobot move speed, 0-100 (default %(default)s)")
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
                    log_timing=args.log_timing)

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
