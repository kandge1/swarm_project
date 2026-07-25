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

MOTION SMOOTHNESS: a trajectory with only one JointTrajectoryPoint at
t=duration lets joint_trajectory_controller interpolate smoothly in
software, but on real hardware mycobot_bridge.py's background loop turns
each interpolated 100Hz sample into its own pymycobot send_angles(...,
speed) call -- an onboard, speed-profiled point-to-point move on the arm's
own MCU, not a raw position write. A fast stream of slightly-different
targets means each onboard move gets superseded by the next before
finishing, which looks like visible discrete jumps rather than one smooth
sweep. Sending several waypoints spaced WAYPOINT_INTERVAL_SEC apart (instead
of one point at the end) gives each intermediate onboard move enough time to
actually get most of the way there before the next target arrives, which is
what actually smooths out the motion -- see also DEFAULT_SPEED lowered in
mycobot_bridge.py for the same reason.

Usage:
  python3 joint_trajectory_test.py                  # squat -> zero -> squat, once
  python3 joint_trajectory_test.py --cycles 5        # repeat 5 times
  python3 joint_trajectory_test.py --duration 4.0    # slower, 4s per move
  python3 joint_trajectory_test.py --to zero         # single move, squat's
                                                      # current position -> zero
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

# How far apart (in time) consecutive waypoints are. See the module
# docstring's MOTION SMOOTHNESS note for why this matters on real hardware.
WAYPOINT_INTERVAL_SEC = 0.25


def make_trajectory(start_radians, target_radians, duration_sec):
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINT_NAMES

    n_waypoints = max(1, round(duration_sec / WAYPOINT_INTERVAL_SEC))
    points = []
    for i in range(1, n_waypoints + 1):
        t = min(i * WAYPOINT_INTERVAL_SEC, duration_sec)
        frac = t / duration_sec if duration_sec > 0 else 1.0
        point = JointTrajectoryPoint()
        point.positions = [
            start_radians[name] + frac * (target_radians[name] - start_radians[name])
            for name in ARM_JOINT_NAMES
        ]
        point.velocities = [0.0] * len(ARM_JOINT_NAMES)
        point.time_from_start.sec = int(t)
        point.time_from_start.nanosec = int((t % 1) * 1e9)
        points.append(point)
    # Make sure the final waypoint lands exactly at duration_sec/target_radians
    # even if duration_sec isn't an exact multiple of WAYPOINT_INTERVAL_SEC.
    points[-1].positions = [target_radians[name] for name in ARM_JOINT_NAMES]
    points[-1].time_from_start.sec = int(duration_sec)
    points[-1].time_from_start.nanosec = int((duration_sec % 1) * 1e9)

    traj.points = points
    return traj


def move_to(io_client, pose_name, duration_sec):
    print(f"[joint_trajectory_test] moving to {pose_name!r} pose over {duration_sec}s...")
    start_radians = io_client.current_joint_positions(ARM_JOINT_NAMES)
    trajectory = make_trajectory(start_radians, POSES_RADIANS[pose_name], duration_sec)
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
    args = parser.parse_args()

    rclpy.init()
    io_client = RobotIOClient()

    try:
        if args.to is not None:
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
