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

    # Spawners are staggered (3.0s, 5.0s, 7.0s) rather than fired together at
    # one TimerAction. With Cyclone DDS (see swarm_network's cyclonedds.xml),
    # each spawner is a short-lived CLI process that must complete SPDP
    # discovery of ros2_control_node's participant before it can call
    # get_node_names_and_namespaces() -- three of them starting in the same
    # instant on the Pi's limited CPU turned into a discovery race, with the
    # losers crashing ("empty node name returned by the RMW layer"). This
    # was never an issue with the default RMW (Fast DDS), only appeared once
    # Cyclone DDS was introduced for cross-machine unicast discovery.
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
        ],
    )
    delayed_arm_group_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_group_controller"],
                output="screen",
            ),
        ],
    )
    delayed_gripper_group_controller = TimerAction(
        period=7.0,
        actions=[
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
        delayed_joint_state_broadcaster,
        delayed_arm_group_controller,
        delayed_gripper_group_controller,
    ])
