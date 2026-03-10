from setuptools import find_packages, setup

package_name = 'lio_specific_pkg_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'joint_io = lio_specific_pkg_ros2.joint_io:main',
            'ik_stream_to_action = lio_specific_pkg_ros2.ik_stream_to_action:main',
            'myp_application = lio_specific_pkg_ros2.myp_application:main',
            'ik_interface_ros2 = lio_specific_pkg_ros2.ik_interface_lio_ros2:main',
            
        ],
    },
)
