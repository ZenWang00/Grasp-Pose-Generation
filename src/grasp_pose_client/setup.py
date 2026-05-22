from setuptools import find_packages, setup
from glob import glob

package_name = 'grasp_pose_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel@todo.todo',
    description='ROS2 client for the remote VLA grasp HTTP server.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_pose_client_node = grasp_pose_client.grasp_pose_client_node:main',
            'capture_replay_node = grasp_pose_client.capture_replay_node:main',
        ],
    },
)
