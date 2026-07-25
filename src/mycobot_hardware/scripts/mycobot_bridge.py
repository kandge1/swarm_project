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
DEFAULT_SPEED = 50  # 0-100, pymycobot's joint/gripper move speed

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


class Bridge:
    def __init__(self, serial_port, baud_rate, speed):
        from pymycobot import MyCobot280  # imported here so --help works without hardware attached

        self.speed = speed
        print(f"[mycobot_bridge] connecting to {serial_port} @ {baud_rate}...")
        self.arm = MyCobot280(serial_port, baud_rate)
        print("[mycobot_bridge] connected.")
        self.state = SharedState(len(JOINT_ORDER))
        self._stop = threading.Event()

    # ---- background thread: owns the arm exclusively ----

    def serial_loop(self):
        """Runs continuously on its own thread for the lifetime of the
        process. Never touched by the socket handler -- this is the only
        code that calls into pymycobot, so a slow/stalled serial call here
        blocks nothing except this loop's own next iteration."""
        print("[mycobot_bridge] serial loop starting")
        while not self._stop.is_set():
            self._serial_read_once()
            self._serial_write_once_if_dirty()

    def _serial_read_once(self):
        # pymycobot's get_* calls commonly return -1 (a truthy int, not
        # None/0/[]) on a communication error/timeout instead of raising --
        # observed for real over serial: `get_angles()` returned a bare int,
        # which crashed here (and took the whole bridge process down with
        # it, since nothing caught it) when iterated as if it were the
        # expected 6-element list. Validate shape explicitly rather than
        # relying on truthiness.
        try:
            t0 = time.monotonic()
            angles_deg = self.arm.get_angles()
            t1 = time.monotonic()
            print(f"[mycobot_bridge] TIMING get_angles() took {(t1 - t0) * 1000:.1f}ms")
            if not isinstance(angles_deg, (list, tuple)) or len(angles_deg) != 6:
                print(f"[mycobot_bridge] WARNING: get_angles() returned "
                      f"{angles_deg!r}, expected a 6-element list -- keeping "
                      f"last known positions for this read")
                angles_deg = None

            t2 = time.monotonic()
            gripper_value = self.arm.get_gripper_value()
            t3 = time.monotonic()
            print(f"[mycobot_bridge] TIMING get_gripper_value() took {(t3 - t2) * 1000:.1f}ms")
            if not isinstance(gripper_value, (int, float)):
                print(f"[mycobot_bridge] WARNING: get_gripper_value() returned "
                      f"{gripper_value!r}, expected a number -- keeping last "
                      f"known gripper position for this read")
                gripper_value = None
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
            self.state.command_dirty = False

        arm_degrees = [math.degrees(p) for p in positions[:6]]
        try:
            t0 = time.monotonic()
            self.arm.send_angles(arm_degrees, self.speed)
            t1 = time.monotonic()
            print(f"[mycobot_bridge] TIMING send_angles() took {(t1 - t0) * 1000:.1f}ms")

            gripper_rad = positions[6]
            gripper_value = gripper_rad_to_value(gripper_rad)
            t2 = time.monotonic()
            self.arm.set_gripper_value(gripper_value, self.speed)
            t3 = time.monotonic()
            print(f"[mycobot_bridge] TIMING set_gripper_value() took {(t3 - t2) * 1000:.1f}ms")
        except Exception as exc:
            print(f"[mycobot_bridge] ERROR during serial write: {exc!r}")

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
            recv_t = time.monotonic()
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self._handle_line(conn, line, recv_t)

    def _handle_line(self, conn, line, recv_t):
        arrival_lag_ms = (time.monotonic() - recv_t) * 1000
        try:
            request = json.loads(line)
        except json.JSONDecodeError as exc:
            print(f"[mycobot_bridge] bad request, ignoring: {exc}")
            return

        cmd = request.get("cmd")
        t_start = time.monotonic()
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

        total_ms = (time.monotonic() - t_start) * 1000
        print(f"[mycobot_bridge] TIMING {cmd!r} id={reply['id']} total={total_ms:.1f}ms "
              f"queued_before_dispatch={arrival_lag_ms:.1f}ms")

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
    args = parser.parse_args()

    bridge = Bridge(args.serial_port, args.baud_rate, args.speed)

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
