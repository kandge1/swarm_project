#!/usr/bin/env python3
"""
collision_contacts.py -- identify which link pairs are self-colliding.

Uses MoveIt's /check_state_validity service, whose response already lists
every contact pair directly (response.contacts) -- no need to toggle the
ACM entry-by-entry the way this script used to. That toggling was a
workaround for moveit_py's is_state_colliding() only returning a bool with
no pair information; the service reports contacts outright, so this is
both simpler and no longer depends on moveit_py (see pick_place.py's module
docstring for why that matters).

Usage:
    python3 collision_contacts.py
    python3 collision_contacts.py --joints '0 0 -0.8 -1.4 0 0.27'  # rad

Interpretation:
  !! ARM-vs-ARM  -> MESH ORIGIN ERROR in URDF (fix rpy in <collision><origin>)
  -- EE-vs-arm   -> Missing SRDF disable_collisions entry
"""

import sys, os, argparse
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_place import GROUP_NAME, HOME_RADIANS, RobotIOClient  # noqa: E402

ARM_LINKS = ["g_base","joint1","joint2","joint3","joint4","joint5","joint6","joint6_flange"]
EE_LINKS  = ["camera_flange","wrist_camera_link","wrist_camera_optical_frame",
             "gripper_base","gripper_left1","gripper_left2","gripper_left3",
             "gripper_right1","gripper_right2","gripper_right3"]


def check(io_client, joint_dict, label):
    valid, contacts = io_client.check_state_validity(joint_dict, group_name=GROUP_NAME)
    print()
    print('[' + label + ']')
    vals = ' | '.join(k + ': ' + format(v, '+.3f') for k, v in joint_dict.items())
    print('  ' + vals)

    if valid is None:
        print('  -> /check_state_validity service call failed')
        return
    if valid:
        print('  -> NOT colliding')
        return

    print('  -> COLLIDING (' + str(len(contacts)) + ' pair(s)):')
    for contact in contacts:
        la, lb = contact.contact_body_1, contact.contact_body_2
        arm_arm = (la in ARM_LINKS) and (lb in ARM_LINKS)
        tag = '!! ARM-vs-ARM -> MESH ORIGIN ERROR in URDF' if arm_arm else '-- EE-vs-arm -> missing SRDF entry'
        print('    ' + la + ' <-> ' + lb + '   ' + tag)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joints', type=str, default=None)
    args = parser.parse_args()

    rclpy.init()
    io_client = RobotIOClient()

    check(io_client, HOME_RADIANS, 'home (all zeros)')

    outer = dict(HOME_RADIANS)
    outer['joint3_to_joint2'] = -0.8
    outer['joint4_to_joint3'] = -1.392
    outer['joint6output_to_joint6'] = 0.268
    check(io_client, outer, 'outer-arc (r=0.240)')

    inner = dict(HOME_RADIANS)
    inner['joint3_to_joint2'] = -0.8
    inner['joint4_to_joint3'] = -2.585
    inner['joint6output_to_joint6'] = 0.454
    check(io_client, inner, 'inner-arc (r=0.170)')

    if args.joints:
        vals = [float(v) for v in args.joints.split()]
        keys = list(HOME_RADIANS.keys())
        custom = {keys[i]: vals[i] for i in range(min(len(keys), len(vals)))}
        check(io_client, custom, 'custom')

    print()
    io_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
