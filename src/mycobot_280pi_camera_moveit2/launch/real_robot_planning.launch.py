from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Workstation-side half of the split real-hardware setup: move_group
    # (planning: IK, OMPL) and rviz only. The URDF is still built with
    # hardware_mode=real so robot_description matches what's running on the
    # robot -- move_group never loads the mycobot_hardware/MyCobotSystem
    # plugin itself (that only happens in ros2_control_node, which runs on
    # the robot side -- see real_robot_hardware.launch.py), so this is safe
    # even though this machine (Jazzy) can't build that Galactic-only plugin.
    #
    # Requires DDS discovery working between this machine and the robot --
    # see swarm_network package / WORKFLOW.md "DDS Unicast Discovery".
    moveit_config = (
        MoveItConfigsBuilder("firefighter")
        .robot_description(file_path="config/firefighter.urdf.xacro", mappings={"hardware_mode": "real"})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(
            get_package_share_directory("mycobot_280pi_camera_moveit2"),
            "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        move_group,
        rviz,
    ])
