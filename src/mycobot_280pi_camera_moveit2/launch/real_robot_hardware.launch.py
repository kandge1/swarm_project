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

    # rmw_cyclonedds_cpp has a known, unfixed upstream discovery-timing race
    # (ros2/rclpy#1448, ros2/ros2#489): a spawner can call
    # get_node_names_and_namespaces() a moment before ros2_control_node's ROS
    # graph metadata has fully propagated over DDS, and rcl hard-fails on
    # that with "empty node name returned by the RMW layer" instead of
    # retrying internally. This is strictly a timing race, not a sign
    # anything is misconfigured -- there is no CycloneDDS XML setting that
    # eliminates it (confirmed against the upstream issue threads); the
    # community workaround is to retry at the call site. Our static unicast
    # peers (see swarm_network's cyclonedds.xml) widen the race window
    # versus LAN multicast, so each spawner is both staggered (3s apart) and
    # retried a few times via a shell loop.
    def retrying_spawner(controller_name, delay):
        return TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=["bash", "-c", (
                        "for i in 1 2 3 4 5; do "
                        f"ros2 run controller_manager spawner {controller_name} && exit 0; "
                        "echo \"[retrying_spawner] attempt $i for "
                        f"{controller_name} failed, retrying in 2s...\"; "
                        "sleep 2; "
                        "done; exit 1"
                    )],
                    output="screen",
                ),
            ],
        )

    delayed_joint_state_broadcaster = retrying_spawner("joint_state_broadcaster", 3.0)
    delayed_arm_group_controller = retrying_spawner("arm_group_controller", 6.0)
    delayed_gripper_group_controller = retrying_spawner("gripper_group_controller", 9.0)

    return LaunchDescription([
        mycobot_bridge,
        rsp,
        ros2_control_node,
        delayed_joint_state_broadcaster,
        delayed_arm_group_controller,
        delayed_gripper_group_controller,
    ])
