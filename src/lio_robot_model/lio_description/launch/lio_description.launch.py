from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  

def generate_launch_description():
    # Paths and arguments
    urdf_file = PathJoinSubstitution([
        FindPackageShare('lio_description'),
        'urdf',
        'lio.urdf.xacro'
    ])

    
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # Declare argument if needed later
        DeclareLaunchArgument('use_gui', default_value='true', description='Launch joint_state_publisher_gui'),

        # Robot Description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisherLIO',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'publish_frequency': 50.0
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('lio_description'),
                'rviz',
                'lio_rviz2.rviz'
            ])]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_gui': LaunchConfiguration('use_gui')}]
        )
    ])
















