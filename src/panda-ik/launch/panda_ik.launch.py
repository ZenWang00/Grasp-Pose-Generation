from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'panda_ik',
        'urdfs',
        'lio_arm_reframed.urdf'
    )

    return LaunchDescription([
        Node(
            package='panda_ik',
            executable='panda_ik_node',
            name='panda_ik',
            parameters=[{'URDF': urdf_path}],
            output='screen'
        )
    ])