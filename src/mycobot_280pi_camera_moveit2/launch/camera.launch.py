import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description="V4L2 device path for the wrist USB camera"
    )

    camera_info_url_arg = DeclareLaunchArgument(
        "camera_info_url",
        default_value="",
        description="URL to camera calibration yaml, e.g. file:///path/to/wrist_camera.yaml"
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="wrist_camera",
        output="screen",
        parameters=[{
            "video_device":    LaunchConfiguration("video_device"),
            "image_size":      [640, 480],
            "camera_frame_id": "wrist_camera_optical_frame",
            "camera_info_url": LaunchConfiguration("camera_info_url"),
        }],
        remappings=[
            ("image_raw",   "/wrist_camera/image_raw"),
            ("camera_info", "/wrist_camera/camera_info"),
        ]
    )

    return launch.LaunchDescription([
        video_device_arg,
        camera_info_url_arg,
        camera_node,
    ])
