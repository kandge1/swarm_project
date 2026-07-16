#!/usr/bin/env python3
"""
collision_contacts.py -- identify which link pairs are self-colliding.

For each candidate pair it temporarily allows it in the ACM, re-runs the
collision check, and if the result changes reports it as a culprit.

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
from pick_place import GROUP_NAME, HOME_RADIANS, build_moveit
from moveit.core.robot_state import RobotState

ARM_LINKS = ["g_base","joint1","joint2","joint3","joint4","joint5","joint6","joint6_flange"]
EE_LINKS  = ["camera_flange","wrist_camera_link","wrist_camera_optical_frame",
             "gripper_base","gripper_left1","gripper_left2","gripper_left3",
             "gripper_right1","gripper_right2","gripper_right3"]

_adj = {
    ("g_base","joint1"),("joint1","joint2"),("joint2","joint3"),
    ("joint3","joint4"),("joint4","joint5"),("joint5","joint6"),("joint6","joint6_flange")
}

PAIRS = [(a,b) for i,a in enumerate(ARM_LINKS) for b in ARM_LINKS[i+1:]
         if (a,b) not in _adj and (b,a) not in _adj]
PAIRS += [(ee,arm) for ee in EE_LINKS for arm in ARM_LINKS]


def find_culprits(mycobot, joint_dict):
    state = RobotState(mycobot.get_robot_model())
    state.set_joint_group_positions(GROUP_NAME, list(joint_dict.values()))
    state.update()
    psm = mycobot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        if not scene.is_state_colliding(state, GROUP_NAME):
            return state, []
    culprits = []
    with psm.read_write() as scene:
        acm = scene.allowed_collision_matrix
        for la, lb in PAIRS:
            acm.set_entry(la, lb, True)
            still = scene.is_state_colliding(state, GROUP_NAME)
            acm.set_entry(la, lb, False)
            if not still:
                culprits.append((la, lb))
    return state, culprits


def check(mycobot, joint_dict, label):
    _, culprits = find_culprits(mycobot, joint_dict)
    print()
    print('[' + label + ']')
    vals = ' | '.join(k + ': ' + format(v, '+.3f') for k, v in joint_dict.items())
    print('  ' + vals)
    if not culprits:
        print('  -> NOT colliding')
        return
    print('  -> COLLIDING (' + str(len(culprits)) + ' pair(s)):')
    for la, lb in culprits:
        arm_arm = (la in ARM_LINKS) and (lb in ARM_LINKS)
        tag = '!! ARM-vs-ARM -> MESH ORIGIN ERROR in URDF' if arm_arm else '-- EE-vs-arm -> missing SRDF entry'
        print('    ' + la + ' <-> ' + lb + '   ' + tag)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joints', type=str, default=None)
    args = parser.parse_args()
    rclpy.init()
    mycobot = build_moveit()

    check(mycobot, HOME_RADIANS, 'home (all zeros)')

    outer = dict(HOME_RADIANS)
    outer['joint3_to_joint2'] = -0.8
    outer['joint4_to_joint3'] = -1.392
    outer['joint6output_to_joint6'] = 0.268
    check(mycobot, outer, 'outer-arc (r=0.240)')

    inner = dict(HOME_RADIANS)
    inner['joint3_to_joint2'] = -0.8
    inner['joint4_to_joint3'] = -2.585
    inner['joint6output_to_joint6'] = 0.454
    check(mycobot, inner, 'inner-arc (r=0.170)')

    if args.joints:
        vals = [float(v) for v in args.joints.split()]
        keys = list(HOME_RADIANS.keys())
        custom = {keys[i]: vals[i] for i in range(min(len(keys), len(vals)))}
        check(mycobot, custom, 'custom')

    print()
    sys.stdout.flush()
    os._exit(0)  # bypass MoveItCpp teardown segfault (known Humble upstream bug)


if __name__ == '__main__':
    main()
