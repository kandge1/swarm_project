#!/usr/bin/env python3

import math
import os
import subprocess
import sys

from moveit.core.robot_state import RobotState

# Allow importing pick_place.py from this directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pick_place import (
    GROUP_NAME,
    POSE_LINK,
    PLANNING_FRAME,
    build_moveit,
)


JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


def print_link_position(state, link_name):
    transform = state.get_global_link_transform(link_name)

    x = float(transform[0, 3])
    y = float(transform[1, 3])
    z = float(transform[2, 3])

    print(f"\n{link_name}")
    print("-" * 45)
    print(f"X = {x: .6f} m   ({x * 100: .2f} cm)")
    print(f"Y = {y: .6f} m   ({y * 100: .2f} cm)")
    print(f"Z = {z: .6f} m   ({z * 100: .2f} cm)")

    return x, y, z


def send_to_gazebo(angles_rad):
    print("\n" + "=" * 70)
    print("GAZEBO EXECUTION")
    print("=" * 70)

    # First check whether the arm controller exists
    check = subprocess.run(
        [
            "ros2",
            "action",
            "list",
        ],
        capture_output=True,
        text=True,
    )

    action_name = (
        "/arm_group_controller/follow_joint_trajectory"
    )

    if action_name not in check.stdout:
        print("\nArm controller is not available.")
        print("\nMake sure gazebo.launch.py is running and check:")
        print("  ros2 control list_controllers")
        print("\nFK results above are still valid.")
        return

    positions = ", ".join(
        f"{angle:.10f}" for angle in angles_rad
    )

    goal = (
        "{trajectory: {"
        "joint_names: ["
        "'joint2_to_joint1', "
        "'joint3_to_joint2', "
        "'joint4_to_joint3', "
        "'joint5_to_joint4', "
        "'joint6_to_joint5', "
        "'joint6output_to_joint6'"
        "], "
        "points: [{"
        f"positions: [{positions}], "
        "time_from_start: {sec: 4}"
        "}]"
        "}}"
    )

    print("\nSending these same joint angles to Gazebo...")

    result = subprocess.run(
        [
            "ros2",
            "action",
            "send_goal",
            action_name,
            "control_msgs/action/FollowJointTrajectory",
            goal,
        ]
    )

    if result.returncode == 0:
        print("\nGazebo command completed.")
    else:
        print("\nGazebo command failed.")


def main():
    print("=" * 70)
    print("MYCOBOT 280 PI - FORWARD KINEMATICS + GAZEBO")
    print("=" * 70)

    print("\nEnter 6 joint angles in DEGREES.")

    print("\nJoint order:")
    for i, name in enumerate(JOINT_NAMES, start=1):
        print(f"  J{i}: {name}")

    print("\nExample:")
    print("  10 -10 10 -10 10 10")

    values = input("\nAngles: ").strip().split()

    if len(values) != 6:
        print("\nERROR: Enter exactly 6 joint angles.")
        return

    try:
        angles_deg = [float(value) for value in values]
    except ValueError:
        print("\nERROR: All angles must be numbers.")
        return

    angles_rad = [
        math.radians(angle)
        for angle in angles_deg
    ]

    print("\nAngles entered:")
    print("-" * 70)

    for i in range(6):
        print(
            f"J{i + 1}: "
            f"{JOINT_NAMES[i]:<25} "
            f"{angles_deg[i]:8.3f} deg   "
            f"{angles_rad[i]:9.6f} rad"
        )

    # ---------------------------------------------------------
    # Load exact MoveIt configuration used by project
    # ---------------------------------------------------------

    print("\nLoading robot model...")

    mycobot = build_moveit()

    robot_model = mycobot.get_robot_model()

    state = RobotState(robot_model)

    state.set_to_default_values()

    state.set_joint_group_positions(
        GROUP_NAME,
        angles_rad,
    )

    state.update()

    print("\nRobot model loaded.")
    print(f"Planning group : {GROUP_NAME}")
    print(f"Planning frame : {PLANNING_FRAME}")
    print(f"End link       : {POSE_LINK}")

    # ---------------------------------------------------------
    # Forward kinematics
    # ---------------------------------------------------------

    joint_model_group = robot_model.get_joint_model_group(
        GROUP_NAME
    )

    link_names = joint_model_group.link_model_names

    print("\n" + "=" * 70)
    print("FORWARD KINEMATICS RESULTS")
    print("=" * 70)

    for link_name in link_names:
        try:
            print_link_position(
                state,
                link_name,
            )

        except Exception as error:
            print(
                f"\nCould not calculate FK for "
                f"{link_name}: {error}"
            )

    # ---------------------------------------------------------
    # Final flange / end-effector position
    # ---------------------------------------------------------

    print("\n" + "=" * 70)
    print("END-EFFECTOR / FLANGE POSITION")
    print("=" * 70)

    transform = state.get_global_link_transform(
        POSE_LINK
    )

    x = float(transform[0, 3])
    y = float(transform[1, 3])
    z = float(transform[2, 3])

    print(f"\nLink: {POSE_LINK}")

    print(
        f"X = {x:.6f} m "
        f"({x * 100:.2f} cm)"
    )

    print(
        f"Y = {y:.6f} m "
        f"({y * 100:.2f} cm)"
    )

    print(
        f"Z = {z:.6f} m "
        f"({z * 100:.2f} cm)"
    )

    print("\nRotation matrix:")

    for row in range(3):
        print(
            f"{transform[row, 0]: .6f}  "
            f"{transform[row, 1]: .6f}  "
            f"{transform[row, 2]: .6f}"
        )

    # ---------------------------------------------------------
    # Move Gazebo
    # ---------------------------------------------------------

    send_to_gazebo(angles_rad)

    print("\n" + "=" * 70)
    print("DONE")
    print("=" * 70)


if __name__ == "__main__":
    main()
