"""Launch the grasp_pose_client node only (does NOT start realsense2_camera).

Bring up the RealSense driver separately, e.g.:

    ros2 launch realsense_camera rs_launch.py \
        align_depth.enable:=true \
        pointcloud.enable:=false

Then:

    ros2 launch grasp_pose_client grasp_pose_client.launch.py \
        server_url:=http://localhost:8765
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_args = [
        DeclareLaunchArgument(
            "server_url",
            default_value="http://localhost:8765",
            description="Base URL of the remote VLA grasp HTTP server.",
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value="/camera/camera/color/image_raw",
            description="sensor_msgs/Image topic for the color stream.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
            description="sensor_msgs/Image topic for the depth stream aligned to color.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="sensor_msgs/CameraInfo topic matched to the color stream.",
        ),
        DeclareLaunchArgument(
            "sync_slop_s",
            default_value="0.2",
            description="ApproximateTimeSynchronizer slop in seconds.",
        ),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value="30",
            description="ApproximateTimeSynchronizer queue size.",
        ),
        DeclareLaunchArgument(
            "request_timeout_s",
            default_value="60.0",
            description="HTTP request timeout for POST /grasp (seconds).",
        ),
        DeclareLaunchArgument(
            "default_top_k",
            default_value="1",
            description="Default number of top grasps to return.",
        ),
        DeclareLaunchArgument(
            "default_num_candidates",
            default_value="1",
            description="Default number of VLM grasp_region_box proposals.",
        ),
        DeclareLaunchArgument(
            "max_snapshot_age_s",
            default_value="2.0",
            description="Reject service calls if the latest synced frame is older than this.",
        ),
        DeclareLaunchArgument(
            "probe_health_on_startup",
            default_value="true",
            description="Call GET /health at startup so misconfiguration shows up early.",
        ),
        DeclareLaunchArgument(
            "gripper_frame_id",
            default_value="camera_color_optical_frame",
            description="End-effector/camera TF frame used for gripper→base TF lookup.",
        ),
        DeclareLaunchArgument(
            "robot_base_frame_id",
            default_value="LIO_base_link",
            description="Robot base TF frame for the gripper→base transform. "
                        "Must match the IK URDF root (LIO_base_link) so that "
                        "Pinocchio feasibility check and execution use the same frame.",
        ),
        DeclareLaunchArgument(
            "log_dir",
            default_value="~/grasp_logs",
            description="Directory for per-run JSONL grasp session logs.",
        ),
        # Static TF: lio_gripper_interface_link → camera_link
        # Hand-eye calibration result (RealSense externally mounted on gripper):
        #   translation: x=-0.10 m, y=0, z=+0.052 m
        #   rotation:    RPY(roll=0, pitch=-pi/2, yaw=0)  →  Ry(-90°)
        #   quaternion:  (qx=0, qy=-0.7071, qz=0, qw=0.7071)
        # Equivalent CLI: static_transform_publisher -0.10 0 0.052 0 -1.5708 0 lio_gripper_interface_link camera_link
        DeclareLaunchArgument("cam_x",  default_value="-0.1"),
        DeclareLaunchArgument("cam_y",  default_value="0.0"),
        DeclareLaunchArgument("cam_z",  default_value="0.052"),
        DeclareLaunchArgument(
            "ik_urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("panda_ik"), "urdfs", "lio_arm_reframed.urdf"
            ]),
            description="URDF for Pinocchio IK feasibility check. Set empty to disable.",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/ik_interface/joint_states_sim",
            description="JointState topic used to seed the Pinocchio IK solver.",
        ),
        DeclareLaunchArgument("cam_qx", default_value="0.0"),
        DeclareLaunchArgument("cam_qy", default_value="-0.7071067811865476"),  # Ry(-90°)
        DeclareLaunchArgument("cam_qz", default_value="0.0"),
        DeclareLaunchArgument("cam_qw", default_value="0.7071067811865476"),
    ]

    # Bridges the robot URDF TF tree (lio_gripper_interface_link) to the RealSense driver TF tree (camera_link).
    # Transform: lio_gripper_interface_link → camera_link  (parent → child)
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gripper_to_camera_link",
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_qx"),
            LaunchConfiguration("cam_qy"),
            LaunchConfiguration("cam_qz"),
            LaunchConfiguration("cam_qw"),
            "lio_gripper_interface_link",  # parent frame (in robot URDF)
            "camera_link",                 # child frame  (published by realsense2_camera driver)
        ],
    )

    grasp_pose_client_node = Node(
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
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "request_timeout_s": LaunchConfiguration("request_timeout_s"),
            "default_top_k": LaunchConfiguration("default_top_k"),
            "default_num_candidates": LaunchConfiguration("default_num_candidates"),
            "max_snapshot_age_s": LaunchConfiguration("max_snapshot_age_s"),
            "probe_health_on_startup": LaunchConfiguration("probe_health_on_startup"),
            "gripper_frame_id": LaunchConfiguration("gripper_frame_id"),
            "robot_base_frame_id": LaunchConfiguration("robot_base_frame_id"),
            # Systematic extrinsic-bias correction (meters) added to the grasp position
            # AFTER transformation into robot_base_frame_id (= LIO_base_link).
            # Axes follow LIO_base_link: +x = arm-forward, +y = left, +z = up.
            # 2026-06-12: reset to zero. The ~(+0.122, 0, −0.22) cmd-vs-stop mismatch
            # this used to compensate was traced to two execution-side bugs, now fixed:
            # (1) panda_ik URDFs placed lio_joint1 0.212 m higher than the platform's
            #     robot_description (dz), and (2) ik_stream_to_action advanced its
            #     stream even when move_joints goals were rejected (dx undershoot).
            # If a small constant residual remains, recalibrate from fresh logs.
            "grasp_offset_base_xyz": [0.0, 0.0, 0.0],
            "ik_urdf_path": LaunchConfiguration("ik_urdf_path"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "log_dir": LaunchConfiguration("log_dir"),
        }],
    )

    return LaunchDescription([*declared_args, camera_static_tf, grasp_pose_client_node])
