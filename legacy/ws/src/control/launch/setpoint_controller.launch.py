from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Setpoint controller that outputs /cmd_vel
        Node(
            package='control',
            executable='setpoint_controller',
            name='setpoint_controller',
            output='screen',
            parameters=[
                {
                    'odom_topic': '/odom',
                    'cmd_vel_topic': '/cmd_vel',
                    'goal_marker_topic': '/goal_marker',
                    'frame_id': 'odom',
                }
            ]
        ),

        # Differential Drive Solver from /cmd_vel to /wheelVel JointState for Isaacsim
        Node(
            package='control',
            executable='diff_drive_kinematics',
            name='diff_drive_kinematics',
            output='screen',
            parameters=[
                {
                    'cmd_vel_topic': '/cmd_vel',
                    'wheel_vel_topic': '/wheelVel',
                    'wheel_radius': 0.14,
                    'wheel_base': 0.413,
                    'left_wheel_joint': 'joint_wheel_left',
                    'right_wheel_joint': 'joint_wheel_right',
                }
            ]
        ),
    ])
