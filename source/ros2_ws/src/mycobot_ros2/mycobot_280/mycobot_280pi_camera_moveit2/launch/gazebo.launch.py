from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Build MoveIt config with use_sim=true so gz_ros2_control/GazeboSimSystem is used
    moveit_config = (
        MoveItConfigsBuilder("firefighter", package_name="mycobot_280pi_camera_moveit2")
        .robot_description(mappings={"use_sim": "true"})
        .to_moveit_configs()
    )

    # Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py",
            )
        ]),
        launch_arguments={
            # Custom world (not the stock empty.sdf) because it adds the
            # Sensors system plugin - without it, gz-sim registers the
            # wrist camera's gz-transport topic but never renders/publishes
            # any frames on it.
            "gz_args": "-r " + os.path.join(
                get_package_share_directory("mycobot_280pi_camera_moveit2"),
                "worlds", "camera_world.sdf",
            ),
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn the robot into Gazebo from the robot_description topic, positioned
    # so g_base -- a fictitious mounting-plate link with no equivalent on the
    # real hardware -- sinks into the table, leaving the real robot (starting
    # at joint1) flush with the table top, matching how it'd actually be
    # bolted down.
    #
    # g_base's link frame IS the spawn root (world->g_base fixed joint has
    # zero offset), and joint1 attaches to g_base via ANOTHER zero-offset
    # fixed joint -- so g_base and joint1 share the exact same origin point.
    # Their meshes were authored to meet right at that shared point:
    #   - G_base.dae (mm-scaled): full mesh spans local z [0, 32]mm before
    #     the URDF <origin xyz="0 0 -0.03"> shift is applied (the accompanying
    #     rpy is a pure Z-rotation, doesn't affect z) -> in the shared root
    #     frame this base plate spans z [-0.030, +0.002] -- its top sits
    #     essentially AT the root origin.
    #   - joint1_pi.dae (already meters, zero <origin>): spans local z
    #     [0.000, 0.073] directly in the shared root frame -- its bottom
    #     sits exactly AT the root origin too.
    # (Both measured with trimesh, which walks each mesh's COLLADA node/
    # matrix transform hierarchy and applies its <unit meter> scale --
    # reading raw vertex arrays directly, as an earlier pass over the
    # gripper meshes did, ignores that hierarchy and gives wrong numbers.)
    #
    # So placing the shared root origin AT the table top (z=0.02, see
    # camera_world.sdf) puts g_base's top and joint1's bottom in the same
    # place (to within the ~2mm the two meshes were authored apart), exactly
    # the "g_base buried, real robot flush" mount this is going for. Note the
    # table itself is only 0.02m thick (camera_world.sdf) while g_base's full
    # depth is 0.032m, so about 1cm of g_base necessarily pokes out the
    # BOTTOM of the table into the floor -- g_base is a fixed (non-dynamic)
    # mount with no collision consequence, so this is purely a cosmetic
    # sliver hidden under the table from any normal viewing angle.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "firefighter", "-z", "0.02"],
        output="screen",
    )

    # ROS 2 core nodes
    # use_sim_time is required on every node here: joint_states/tf/etc. from
    # Gazebo are stamped with sim time (via /clock), and without this a node
    # comparing its own wall-clock time against those stamps sees them as
    # perpetually stale — e.g. move_group's trajectory_execution_manager
    # rejecting every execution with "couldn't receive full current joint
    # state within 1s" even though joint_states is publishing fine.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # Bridge Gazebo camera topics to ROS 2
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/wrist_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        output="screen",
    )

    # Controller spawners
    # Small delay so gz_sim/gz_ros2_control has started before spawners begin polling
    # for /controller_manager; the spawners themselves already retry until the service
    # is up, so this doesn't need to cover the full startup time. Keeping this short
    # matters: with no controller yet claiming the position command interface, the arm
    # is uncommanded and sags under gravity until a controller activates and latches
    # onto whatever pose it finds as its hold point — a long delay here bakes in a
    # sagged "home" position.
    delayed_controllers = TimerAction(
        period=2.0,
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
        gz_sim,
        rsp,
        spawn_robot,
        camera_bridge,
        move_group,
        rviz,
        delayed_controllers,
    ])
