from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Use xacro instead of static URDF
    xacro_file = PathJoinSubstitution([
        FindPackageShare('lio_description'),
        'urdf',
        'lio.urdf.xacro'
    ])

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Launch argument handles (runtime-resolved)
    physical_robot = LaunchConfiguration('physical_robot')
    
    virtual_robot = LaunchConfiguration('virtual_robot')
    use_joy = LaunchConfiguration('use_joy')
    use_gui = LaunchConfiguration('use_gui')  # kept for future GUI nodes
    panda_ik = LaunchConfiguration('panda_ik')
    output_topic = LaunchConfiguration('output')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('virtual_robot', default_value='true'),
        DeclareLaunchArgument('physical_robot', default_value='false'),
        DeclareLaunchArgument('use_joy', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('output', default_value='/ik_interface/joint_states_sim'),    #can be /ik_interface/joint_states_sim in simulation or /ik_interface/joint_states_lio for physical robot
        DeclareLaunchArgument('panda_ik', default_value='true'),
        DeclareLaunchArgument('node_start_delay', default_value='1.0'),



        #Myp Application Node
        # Node(
        #     package='lio_specific_pkg_ros2',
        #     executable='myp_application', 
        #     name='myp_application',
        #     output='screen'),



        #IK Interface Node
        Node(
            package='lio_specific_pkg_ros2',
            executable='ik_interface_ros2',
            name='ik_interface_ros2',
            output='screen',
            parameters=[{
                'physical_robot': ParameterValue(LaunchConfiguration('physical_robot'), value_type=bool)
            }]

            # parameters=[{'physical_robot': physical_robot}]
        ),


        # Panda IK Node (conditionally started)
        Node(
            package='panda_ik',
            executable='panda_ik_teleop',
            name='panda_ik',
            namespace='panda_ik',
            output='screen',
            condition=IfCondition(panda_ik),
            parameters=[{
                'URDF': PathJoinSubstitution([
                    FindPackageShare('panda_ik'),
                    'urdfs',
                    'lio_arm.urdf'
                ]),
                'weighted_pose': False
            }]
        ),



        # Robot State Publisher (simulation only: physical_robot == false)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='lio_state_publisher',
            output='screen',
            condition=UnlessCondition(physical_robot),
            parameters=[{
                'robot_description': robot_description_content,
                'publish_frequency': 1000.0,
                'ignore_timestamp': True
            }],
            remappings=[('joint_states', output_topic)]
        ),

        # SpaceMouse input (via spacenav_node, publishes /spacenav/joy etc.)
        Node(
            package='spacenav',
            executable='spacenav_node',
            name='spacenav_node',
            output='screen',
            condition=IfCondition(use_joy),
        ),
        # # Joystick input xbox(optional)
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     output='screen',
        #     parameters=[{'dev': '/dev/input/js0'}],
        #     condition=IfCondition(use_joy),
        # ),

        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_joy',
        #     output='screen',
        #     condition=IfCondition(use_joy),
        # ),

        Node(
            package='lio_teleop',
            executable='joy_driver',
            name='joy_driver',
            output='screen',
            condition=IfCondition(use_joy),
            remappings=[('/joy', '/spacenav/joy')],
        ),

        # Rail Follower Node — projects joystick velocity onto pre-grasp path
        Node(
            package='rail_follower',
            executable='rail_follower_node',
            name='rail_follower',
            output='screen',
            condition=IfCondition(use_joy),
            parameters=[{
                'pre_grasp_offset_m': 0.15,
                'path_num_points': 200,
                'control_rate_hz': 50.0,
                'rail_toggle_button': 1,
                'base_frame_id': 'LIO_robot_base_link',
                'gripper_frame_id': 'lio_gripper_joint',
                'joy_topic': '/spacenav/joy',
                'grasp_topic': '/grasp_pose_client/best_grasp',
            }],
        ),

        # Velocity Controller Node — consumes filtered velocity from rail_follower
        Node(
            package='lio_teleop',
            executable='velocity_controller',
            name='velocity_controller',
            output='screen',
            remappings=[('/commanded_vel', '/commanded_vel_filtered')],
        ),


        #RViz (only when virtual robot visualization is enabled)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            condition=IfCondition(virtual_robot),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('lio_teleop'),
                'rviz',
                'lio_teleop.rviz'
            ])]
        )


    ])

















# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition
# from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():
#     # Use xacro instead of static URDF
#     xacro_file = PathJoinSubstitution([
#         FindPackageShare('lio_description'),
#         'urdf',
#         'lio.urdf.xacro'
#     ])

#     robot_description_content = ParameterValue(
#         Command(['xacro ', xacro_file]),
#         value_type=str
#     )

#     return LaunchDescription([
#         # Arguments
#         DeclareLaunchArgument('virtual_robot', default_value='true'),
#         DeclareLaunchArgument('physical_robot', default_value='false'),
#         DeclareLaunchArgument('use_joy', default_value='true'),
#         DeclareLaunchArgument('use_gui', default_value='true'),
#         DeclareLaunchArgument('output', default_value='/ik_interface/joint_states_sim'),
#         DeclareLaunchArgument('panda_ik', default_value='true'),
#         DeclareLaunchArgument('node_start_delay', default_value='1.0'),

#         # IK Interface Node
#         Node(
#             package='lio_specific_pkg',
#             executable='ik_interface',
#             name='ik_interface',
#             output='screen',
#             parameters=[{'physical_robot': LaunchConfiguration('physical_robot')}]
#         ),

#         # Panda IK Node (conditionally started)
#         Node(
#             package='panda_ik',
#             executable='panda_ik_teleop',
#             name='panda_ik',
#             namespace='panda_ik', 
#             output='screen',
#             condition=IfCondition(LaunchConfiguration('panda_ik')),
#             parameters=[{
#                 'URDF': PathJoinSubstitution([
#                     FindPackageShare('panda_ik'),
#                     'urdfs',
#                     'lio_arm.urdf'
#                 ]),
#                 'weighted_pose': False
#             }]
#         ),

#         # Velocity Controller Node
#         Node(
#             package='lio_teleop',
#             executable='velocity_controller',
#             name='velocity_controller',
#             output='screen'
#         ),

#         # Robot State Publisher
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='lio_state_publisher',
#             output='screen',
#             condition=IfCondition(LaunchConfiguration('virtual_robot')),
#             parameters=[{
#                 'robot_description': robot_description_content,
#                 'publish_frequency': 1000.0,
#                 'ignore_timestamp': True
#             }],
#             # remappings=[('joint_states', LaunchConfiguration('output'))]
#             remappings=[('joint_states', '/ik_interface/joint_states_sim')]
#         ),

#         # # Joint State Publisher GUI (manually manipulate joints)
#         # Node(
#         #     package='joint_state_publisher_gui',
#         #     executable='joint_state_publisher_gui',
#         #     name='joint_state_publisher_gui',
#         #     output='screen',
#         #     condition=IfCondition(LaunchConfiguration('use_gui')),
#         #     remappings=[('joint_states', LaunchConfiguration('output'))]
#         # ),

#         # RViz
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz',
#             output='screen',
#             condition=IfCondition(LaunchConfiguration('virtual_robot')),
#             arguments=['-d', PathJoinSubstitution([
#                 FindPackageShare('lio_teleop'),
#                 'rviz',
#                 'lio_teleop.rviz'
#             ])]
#         ),

#         # Joystick input
#         Node(
#             package='teleop_twist_joy',
#             executable='teleop_node',
#             name='teleop_joy',
#             output='screen',
#             condition=IfCondition(LaunchConfiguration('use_joy')),
#         ),

#         Node(
#             package='joy',
#             executable='joy_node',
#             name='joy_node',
#             output='screen',
#             parameters=[{'dev': '/dev/input/js0'}],
#             condition=IfCondition(LaunchConfiguration('use_joy')),
#         ),

#         Node(
#             package='lio_teleop',
#             executable='joy_driver',
#             name='joy_driver',
#             output='screen',
#             condition=IfCondition(LaunchConfiguration('use_joy')),
#         )
#     ])
