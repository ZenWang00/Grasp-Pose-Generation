"""One-shot launch: capture_replay + grasp_pose_client.

For end-to-end smoke testing without a real RealSense. Example:

    ros2 launch grasp_pose_client grasp_pose_client_with_replay.launch.py \\
        capture_dir:=$HOME/vla-grasp-server/captures/20260417_120019 \\
        server_url:=http://localhost:8765

In another shell, trigger:

    ros2 service call /grasp_pose_client/request_grasp \\
        grasp_pose_client_msgs/srv/RequestGrasp \\
        "{task_spec: 'Target: the blue bottle', top_k: 1}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(
            "capture_dir",
            description="Path to a capture folder with camera_data.npy + color_preview.jpg.",
        ),
        DeclareLaunchArgument(
            "server_url",
            default_value="http://localhost:8765",
            description="Base URL of the remote VLA grasp HTTP server.",
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
        DeclareLaunchArgument("sync_slop_s", default_value="0.05"),
        DeclareLaunchArgument("request_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("default_top_k", default_value="1"),
        DeclareLaunchArgument("default_num_candidates", default_value="1"),
    ]

    replay_node = Node(
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
    )

    client_node = Node(
        package="grasp_pose_client",
        executable="grasp_pose_client_node",
        name="grasp_pose_client",
        output="screen",
        parameters=[{
            "server_url": LaunchConfiguration("server_url"),
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "sync_slop_s": LaunchConfiguration("sync_slop_s"),
            "request_timeout_s": LaunchConfiguration("request_timeout_s"),
            "default_top_k": LaunchConfiguration("default_top_k"),
            "default_num_candidates": LaunchConfiguration("default_num_candidates"),
            # The replayed depth is already aligned with color (same array, same K),
            # so let the client accept slightly older snapshots while the replay
            # publisher warms up.
            "max_snapshot_age_s": 5.0,
            "probe_health_on_startup": True,
        }],
    )

    return LaunchDescription([*declared, replay_node, client_node])
