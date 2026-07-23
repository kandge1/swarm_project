#!/usr/bin/env python3
"""
Reset the mycobot 280 arm to its designated home pose.

Home pose was specified in degrees:
  joint2_to_joint1:        2
  joint3_to_joint2:       41
  joint4_to_joint3:      -89
  joint5_to_joint4:       48
  joint6_to_joint5:       -2
  joint6output_to_joint6:  0

Converted to radians below (MoveIt/ROS use radians throughout). Note this
closely matches config/initial_positions.yaml already in the repo -- that
file has the same home pose, just wired to MoveIt's "fake" hardware
interface rather than the live Gazebo/gz_ros2_control system this script
targets.

Delegates all planning/execution to pick_place.py's RobotIOClient/go_home --
no moveit_py here, just plain ROS2 services/actions against an externally-
launched move_group (see pick_place.py's module docstring for why).
"""

import sys, os

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_place import RobotIOClient, go_home  # noqa: E402


def main():
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])

    io_client = RobotIOClient()

    if go_home(io_client):
        print("Arm reset to home pose.")
    else:
        print("Failed to reset arm to home pose.")

    io_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
