"""Combined launch: RealSense camera + grasp_pose_client + lio_teleoperation.

Usage (real robot, all defaults apply):

    ros2 launch lio_teleop full.launch.py

Override server URL (e.g. server on a remote machine):

    ros2 launch lio_teleop full.launch.py server_url:=http://192.168.x.x:8765
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(
            'physical_robot',
            default_value='false',
            description='Connect to the physical robot (true) or simulate (false).',
        ),
        DeclareLaunchArgument(
            'virtual_robot',
            default_value='true',
            description='Enable RViz visualisation (not needed on real hardware).',
        ),
        DeclareLaunchArgument(
            'server_url',
            default_value='http://localhost:8765',
            description='Base URL of the VLA grasp HTTP server.',
        ),
    ]

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_launch.py',
        ]),
        launch_arguments={
            'align_depth.enable':          'true',
            'pointcloud.enable':           'false',
            'depth_module.depth_profile':  '848,480,6',
            'rgb_camera.color_profile':    '848,480,6',
        }.items(),
    )

    grasp_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('grasp_pose_client'), '/launch/grasp_pose_client.launch.py',
        ]),
        launch_arguments={
            'server_url':     LaunchConfiguration('server_url'),
            'sync_slop_s':    '0.2',
            'sync_queue_size': '30',
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('lio_teleop'), '/launch/lio_teleoperation.launch.py',
        ]),
        launch_arguments={
            'physical_robot': LaunchConfiguration('physical_robot'),
            'virtual_robot':  LaunchConfiguration('virtual_robot'),
        }.items(),
    )

    return LaunchDescription([*declared, realsense, grasp_client, teleop])
