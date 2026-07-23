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


class Bridge:
    def __init__(self, serial_port, baud_rate, speed):
        from pymycobot import MyCobot280  # imported here so --help works without hardware attached

        self.speed = speed
        print(f"[mycobot_bridge] connecting to {serial_port} @ {baud_rate}...")
        self.arm = MyCobot280(serial_port, baud_rate)
        print("[mycobot_bridge] connected.")

    def read_state(self):
        angles_deg = self.arm.get_angles() or [0.0] * 6
        positions = [math.radians(a) for a in angles_deg]

        gripper_value = self.arm.get_gripper_value()
        gripper_rad = gripper_value_to_rad(gripper_value) if gripper_value is not None else 0.0
        positions.append(gripper_rad)

        velocities = [0.0] * len(JOINT_ORDER)  # see module docstring
        efforts = [0.0] * len(JOINT_ORDER)      # see module docstring

        return {"positions": positions, "velocities": velocities, "efforts": efforts}

    def write_command(self, positions):
        if len(positions) != len(JOINT_ORDER):
            print(f"[mycobot_bridge] WARNING: expected {len(JOINT_ORDER)} "
                  f"positions, got {len(positions)} -- ignoring write")
            return

        arm_degrees = [math.degrees(p) for p in positions[:6]]
        self.arm.send_angles(arm_degrees, self.speed)

        gripper_rad = positions[6]
        gripper_value = gripper_rad_to_value(gripper_rad)
        self.arm.set_gripper_value(gripper_value, self.speed)

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

        conn.sendall((json.dumps(reply) + "\n").encode())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baud-rate", type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="pymycobot move speed, 0-100 (default %(default)s)")
    args = parser.parse_args()

    bridge = Bridge(args.serial_port, args.baud_rate, args.speed)

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
        server.close()
        if os.path.exists(args.socket_path):
            os.remove(args.socket_path)


if __name__ == "__main__":
    main()
