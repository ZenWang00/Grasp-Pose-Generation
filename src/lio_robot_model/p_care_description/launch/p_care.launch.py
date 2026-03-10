from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('p_care_description'),
        'urdf',
        'p_care.urdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Start RViz automatically'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': ['/pcare_joint_states']}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=LaunchConfiguration('rviz'),
            arguments=['-d', os.path.join(
                get_package_share_directory('urdf_tutorial'),
                'rviz',
                'urdf.rviz'
            )]
        )
    ])

