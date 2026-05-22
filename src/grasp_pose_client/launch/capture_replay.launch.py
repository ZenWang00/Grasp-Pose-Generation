"""Launch the capture replay node only.

    ros2 launch grasp_pose_client capture_replay.launch.py \\
        capture_dir:=$HOME/vla-grasp-server/captures/20260417_120019
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "capture_dir",
            description="Path to a capture folder with camera_data.npy + color_preview.jpg.",
        ),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("frame_id", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument(
            "color_topic", default_value="/camera/camera/color/image_raw"
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/camera/color/camera_info"
        ),
        Node(
            package="grasp_pose_client",
            executable="capture_replay_node",
            name="capture_replay",
            output="screen",
            parameters=[{
                "capture_dir": LaunchConfiguration("capture_dir"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "frame_id": LaunchConfiguration("frame_id"),
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            }],
        ),
    ])
