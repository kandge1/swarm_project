from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def generate_launch_description():
    # Build MoveIt config with hardware_mode=real so mycobot_hardware/MyCobotSystem is used
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

    # mycobot_bridge.py owns the actual serial connection to the physical arm
    # (pymycobot -- see mycobot_hardware/scripts/mycobot_bridge.py). It must
    # already be listening on its Unix domain socket before ros2_control_node
    # activates mycobot_hardware/MyCobotSystem, which connects to it in
    # on_activate() -- see the delayed_controllers TimerAction below.
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

    # Real hardware runs on wall-clock time -- no /clock topic, no
    # use_sim_time (unlike gazebo.launch.py, which needs it because Gazebo
    # stamps /joint_states with sim time).
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
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
        # Resolved via get_package_share_directory rather than
        # moveit_config.package_path: that attribute doesn't exist on ROS2
        # Galactic's MoveItConfigs (which the physical robot runs) -- it's a
        # newer addition. This works identically on every distro.
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

    # Small delay so mycobot_bridge.py has time to open the real serial
    # connection and ros2_control_node has time to load mycobot_hardware/
    # MyCobotSystem before spawners start polling /controller_manager -- same
    # reasoning as gazebo.launch.py's delayed_controllers, slightly longer
    # since a real serial handshake is generally slower than Gazebo's
    # in-process startup.
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
        move_group,
        rviz,
        delayed_controllers,
    ])
