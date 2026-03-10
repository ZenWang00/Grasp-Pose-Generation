from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lio_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srajapakshe',
    maintainer_email='shalutha321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_driver = lio_teleop.joy_driver:main',
            'velocity_controller = lio_teleop.velocity_controller:main',
            'lio_teleop_joy = lio_teleop.lio_teleop_joy:main'
        ],
    },
)


