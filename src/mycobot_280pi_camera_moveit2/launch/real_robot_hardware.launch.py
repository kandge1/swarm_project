from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def generate_launch_description():
    # Robot-side half of the split real-hardware setup: bridge, robot_state_publisher,
    # ros2_control_node, and controller spawners only. Planning (move_group) and
    # visualization (rviz) run on the workstation instead -- see
    # real_robot_planning.launch.py -- talking to this node over DDS (see
    # swarm_network's cyclonedds.xml for the unicast peer setup this requires).
    moveit_config = (
        MoveItConfigsBuilder("firefighter")
        .robot_description(file_path="config/firefighter.urdf.xacro", mappings={"hardware_mode": "real"})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("mycobot_280pi_camera_moveit2"),
        "config", "ros2_controllers.yaml",
    )

    bridge_path = os.path.join(
        get_package_prefix("mycobot_hardware"), "lib", "mycobot_hardware", "mycobot_bridge.py",
    )
    mycobot_bridge = ExecuteProcess(
        cmd=[bridge_path],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    delayed_controllers = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_group_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_group_controller"],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        mycobot_bridge,
        rsp,
        ros2_control_node,
        delayed_controllers,
    ])
