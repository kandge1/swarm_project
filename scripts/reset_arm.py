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
"""

import math

import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder


GROUP_NAME = "arm_group"

HOME_DEGREES = {
    "joint2_to_joint1": 2,
    "joint3_to_joint2": 41,
    "joint4_to_joint3": -89,
    "joint5_to_joint4": 48,
    "joint6_to_joint5": -2,
    "joint6output_to_joint6": 0,
}

HOME_RADIANS = {name: math.radians(deg) for name, deg in HOME_DEGREES.items()}


def _floatify_joint_limits(config_dict):
    try:
        joint_limits = config_dict["robot_description_planning"]["joint_limits"]
    except KeyError:
        return
    for limits in joint_limits.values():
        for key in ("max_velocity", "max_acceleration", "max_position", "min_position"):
            if key in limits and isinstance(limits[key], int):
                limits[key] = float(limits[key])


def build_moveit():
    moveit_config = (
        MoveItConfigsBuilder("firefighter", package_name="mycobot_280pi_camera_moveit2")
        .to_moveit_configs()
    )
    config_dict = moveit_config.to_dict()
    config_dict["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    config_dict["plan_request_params"] = {
        "planning_time": 10.0,
        "planning_pipeline": "ompl",
        "max_velocity_scaling_factor": 1.0,
        "max_acceleration_scaling_factor": 1.0,
    }
    _floatify_joint_limits(config_dict)
    return MoveItPy(node_name="reset_arm", config_dict=config_dict)


def main():
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])

    mycobot = build_moveit()
    arm = mycobot.get_planning_component(GROUP_NAME)

    robot_model = mycobot.get_robot_model()
    goal_state = RobotState(robot_model)
    goal_state.set_joint_group_positions(GROUP_NAME, list(HOME_RADIANS.values()))
    goal_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)

    plan_result = arm.plan()
    if not plan_result:
        print("Planning to home pose FAILED.")
        del mycobot
        rclpy.shutdown()
        return

    print("Executing joint-space move to home pose...")
    mycobot.execute(plan_result.trajectory, controllers=[])
    print("Arm reset to home pose.")

    del mycobot
    rclpy.shutdown()


if __name__ == "__main__":
    main()