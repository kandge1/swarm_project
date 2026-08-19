#!/usr/bin/env python3
"""Minimal gripper open/close check, straight through pymycobot.

Talks to the arm's serial port directly, so the ros2_control bridge
(mycobot_bridge.py) MUST be stopped first -- pymycobot claims the port
exclusively and two processes on it at once produce garbled reads. Same
constraint as ff_verify.py.

Port is /dev/ttyAMA0, not /dev/serial0: on a Pi 4 /dev/serial0 is a symlink to
the mini UART (ttyS0) unless Bluetooth has been disabled in /boot/config.txt,
so opening serial0 can succeed while pointing at a port the arm isn't on --
commands then go nowhere and reads return stale values with no error raised.
"""

import time

from pymycobot.mycobot280 import MyCobot280

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 1000000

# pymycobot's set_gripper_state(flag, speed): flag 0 = open, 1 = close,
# speed is 0-100.
GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1
SPEED = 100


def main():
    arm = MyCobot280(SERIAL_PORT, BAUD_RATE)
    for _ in range(2):
        arm.set_gripper_state(GRIPPER_OPEN, SPEED)
        time.sleep(1)
        arm.set_gripper_state(GRIPPER_CLOSE, SPEED)
        time.sleep(1)


if __name__ == "__main__":
    main()
