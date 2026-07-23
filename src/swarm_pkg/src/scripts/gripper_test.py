from pymycobot.mycobot280 import MyCobot280
import time
arm=MyCobot280(“/dev/serial0”,1000000)
for i in range(2):
    arm.set_gripper_state(0,100)
    time.sleep(1)
    arm.set_gripper_state(1,100)
    time.sleep(1)
