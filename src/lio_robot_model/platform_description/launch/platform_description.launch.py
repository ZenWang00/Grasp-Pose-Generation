from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration, EnvironmentVariable, Command, PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Use environment variable ROBOT_BATCH, default to 'default'
    robot_batch = EnvironmentVariable('ROBOT_BATCH', default_value='default')

    # Path to xacro file, e.g., urdf/platform_A.urdf.xacro
    xacro_file = PathJoinSubstitution([
        FindPackageShare('platform_description'),
        'urdf',
        ['platform_', robot_batch, '.urdf.xacro']
    ])

    return LaunchDescription([
        # Declare an argument (optional if you want to override with `--ros-args`)
        DeclareLaunchArgument(
            'robot_batch',
            default_value='default',
            description='Platform batch ID (used in xacro filename)'
        ),

        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_platform',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', xacro_file]),
                'publish_frequency': 50.0
            }]
        ),

        # Joint State Publisher node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_platform',
            output='screen',
            parameters=[{'rate': 50.0}]
        )
    ])

