#!/usr/bin/env python3
"""
Send raw joint-space trajectories straight to arm_group_controller's
FollowJointTrajectory action server -- no IK, no OMPL, no /plan_kinematic_path,
no move_group involvement at all. Reuses pick_place.py's RobotIOClient.arm_execute()
(already goes directly to the controller action server; see its docstring),
just builds the JointTrajectory here instead of getting it from a planner.

WHY THIS SCRIPT EXISTS: pick_place.py's pre-grasp/pre-place moves depend on
OMPL + IK succeeding, which is a separate, not-yet-solved problem on real
hardware (uncalibrated pick/place coordinates -- see PROJECT_CONTEXT.md).
This script isolates just "can the arm smoothly track a known-good joint
trajectory" from "can IK/OMPL find one" -- useful for confirming the
mycobot_bridge.py / DDS / controller pipeline moves the arm correctly before
debugging planning separately.

Two named poses, all six arm joints (gripper untouched):
  zero  -- all joints at 0 degrees.
  squat -- the project's designated home/rest pose (matches reset_arm.py /
           pick_place.py's HOME_RADIANS / config/initial_positions.yaml).

--joints/--degrees additionally allow an ARBITRARY target, which is what
isolates the currently-unsolved failure (2026-07-26): pick_place.py's
pre-grasp step gets its goal accepted by arm_group_controller and then the
arm doesn't move. That goal is a 21-waypoint, 9.14s, 0.2 rad/s trajectory
from MoveIt; every trajectory confirmed to move this arm has been a
1-waypoint hand-built one. Sending the SAME final pose as a 1-waypoint goal
separates the pose from the trajectory shape:

  python3 joint_trajectory_test.py --degrees 104.74 -41.17 -46.49 -2.35 0 104.74

  arm MOVES     -> the pose is reachable and the write path is fine; the
                   failure is in executing multi-waypoint streamed
                   trajectories through mycobot_bridge.py's point-to-point
                   send_angles() API.
  arm DOESN'T   -> the pose itself is the problem (pymycobot silently
                   refusing an out-of-range angle, or a physical/servo
                   limit); nothing to do with MoveIt, DDS or goal count.

Usage:
  python3 joint_trajectory_test.py                  # squat -> zero -> squat, once
  python3 joint_trajectory_test.py --cycles 5        # repeat 5 times
  python3 joint_trajectory_test.py --duration 4.0    # slower, 4s per move
  python3 joint_trajectory_test.py --to zero         # single move, squat's
                                                      # current position -> zero
  python3 joint_trajectory_test.py --degrees 104.74 -41.17 -46.49 -2.35 0 104.74
"""

import argparse
import math
import os
import sys
import time

import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_place import RobotIOClient, HOME_RADIANS  # noqa: E402

ARM_JOINT_NAMES = list(HOME_RADIANS.keys())

POSES_RADIANS = {
    "zero": {name: 0.0 for name in ARM_JOINT_NAMES},
    "squat": dict(HOME_RADIANS),
}


def make_trajectory(target_radians, duration_sec):
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = [target_radians[name] for name in ARM_JOINT_NAMES]
    point.velocities = [0.0] * len(ARM_JOINT_NAMES)
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
    traj.points = [point]
    return traj


def move_to(io_client, pose_name, duration_sec, target_radians=None):
    print(f"[joint_trajectory_test] moving to {pose_name!r} pose over {duration_sec}s...")
    if target_radians is None:
        target_radians = POSES_RADIANS[pose_name]
    trajectory = make_trajectory(target_radians, duration_sec)
    ok = io_client.arm_execute(trajectory)
    if ok:
        print(f"[joint_trajectory_test] reached {pose_name!r} (controller reported success -- "
              f"visually confirm the arm actually moved).")
    else:
        print(f"[joint_trajectory_test] FAILED moving to {pose_name!r}")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cycles", type=int, default=1,
                        help="number of squat<->zero round trips (default 1)")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="seconds allotted per move (default 3.0)")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="seconds to pause at each pose before the next move (default 1.0)")
    parser.add_argument("--to", choices=["zero", "squat"], default=None,
                        help="single move to this pose instead of a squat<->zero cycle")
    parser.add_argument("--joints", type=float, nargs=6, default=None,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="single move to an arbitrary target, 6 values in "
                             "RADIANS, in arm_group_controller joint order "
                             f"({', '.join(ARM_JOINT_NAMES)})")
    parser.add_argument("--degrees", type=float, nargs=6, default=None,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="same as --joints but in DEGREES")
    args = parser.parse_args()

    if args.joints is not None and args.degrees is not None:
        parser.error("pass --joints or --degrees, not both")

    explicit = None
    if args.degrees is not None:
        explicit = [math.radians(d) for d in args.degrees]
    elif args.joints is not None:
        explicit = list(args.joints)

    rclpy.init()
    io_client = RobotIOClient()

    try:
        if explicit is not None:
            target = dict(zip(ARM_JOINT_NAMES, explicit))
            print(f"[joint_trajectory_test] explicit target (rad): "
                  f"{[round(v, 4) for v in explicit]}")
            print(f"[joint_trajectory_test] explicit target (deg): "
                  f"{[round(math.degrees(v), 2) for v in explicit]}")
            move_to(io_client, "explicit", args.duration, target_radians=target)
        elif args.to is not None:
            move_to(io_client, args.to, args.duration)
        else:
            for i in range(args.cycles):
                print(f"[joint_trajectory_test] cycle {i + 1}/{args.cycles}")
                move_to(io_client, "zero", args.duration)
                time.sleep(args.settle)
                move_to(io_client, "squat", args.duration)
                time.sleep(args.settle)
    finally:
        io_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
