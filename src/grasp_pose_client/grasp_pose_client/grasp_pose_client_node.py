"""ROS2 client for the remote VLA grasp HTTP server.

Subscribes to the standard ``realsense2_camera`` topic triple
(color + aligned depth + camera_info), caches the latest time-synced snapshot,
and on each ``RequestGrasp.srv`` call posts that snapshot to the grasp server.
The server's top-K poses are returned both as the service response and as
``geometry_msgs/PoseArray`` / ``geometry_msgs/PoseStamped`` topics for RViz.
"""
from __future__ import annotations

import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger

import message_filters
from cv_bridge import CvBridge

from grasp_pose_client_msgs.srv import RequestGrasp

from .http_client import GraspServerError, get_health, poll_capture_request, poll_publish, post_grasp, upload_capture
from .image_conversion import (
    ImageConversionError,
    camera_info_to_K_json,
    color_msg_to_png_bytes,
    depth_msg_to_meters_npy_bytes,
    ensure_shape_match,
)


DEFAULT_COLOR_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
DEFAULT_SERVER_URL = "http://localhost:8765"


class GraspPoseClientNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_pose_client")

        # ROS2 parameters -------------------------------------------------------------
        self.declare_parameter("server_url", DEFAULT_SERVER_URL)
        self.declare_parameter("color_topic", DEFAULT_COLOR_TOPIC)
        self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
        self.declare_parameter("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_s", 0.05)
        self.declare_parameter("request_timeout_s", 60.0)
        self.declare_parameter("default_top_k", 1)
        self.declare_parameter("default_num_candidates", 1)
        self.declare_parameter("default_provider", "")
        self.declare_parameter("default_model", "")
        self.declare_parameter("max_snapshot_age_s", 2.0)
        self.declare_parameter("probe_health_on_startup", True)
        self.declare_parameter("gripper_frame_id", "camera_color_optical_frame")
        self.declare_parameter("robot_base_frame_id", "LIO_robot_base_link")
        self.declare_parameter("tf_timeout_s", 0.2)
        # Constant correction (meters) added to every grasp position AFTER it has been
        # transformed into the robot base frame. Use this to absorb a systematic
        # hand-eye / extrinsic bias. Axes follow LIO_robot_base_link:
        #   +x / -x , +y = toward robot (back) / -y = forward , +z = up.
        self.declare_parameter("grasp_offset_base_xyz", [0.0, 0.0, 0.0])

        self._server_url: str = self.get_parameter("server_url").value
        self._sync_queue_size: int = int(self.get_parameter("sync_queue_size").value)
        self._sync_slop_s: float = float(self.get_parameter("sync_slop_s").value)
        self._request_timeout_s: float = float(self.get_parameter("request_timeout_s").value)
        self._default_top_k: int = int(self.get_parameter("default_top_k").value)
        self._default_num_candidates: int = int(self.get_parameter("default_num_candidates").value)
        self._default_provider: str = str(self.get_parameter("default_provider").value)
        self._default_model: str = str(self.get_parameter("default_model").value)
        self._max_snapshot_age_s: float = float(self.get_parameter("max_snapshot_age_s").value)
        self._gripper_frame_id: str = str(self.get_parameter("gripper_frame_id").value)
        self._robot_base_frame_id: str = str(self.get_parameter("robot_base_frame_id").value)
        self._tf_timeout_s: float = float(self.get_parameter("tf_timeout_s").value)
        _off = list(self.get_parameter("grasp_offset_base_xyz").value or [0.0, 0.0, 0.0])
        if len(_off) != 3:
            self.get_logger().warn(
                f"grasp_offset_base_xyz must have 3 elements, got {_off}; ignoring."
            )
            _off = [0.0, 0.0, 0.0]
        self._grasp_offset_base: np.ndarray = np.array(_off, dtype=np.float64)

        # Internal state --------------------------------------------------------------
        self._bridge = CvBridge()
        self._snapshot_lock = threading.Lock()
        self._latest: Optional[
            tuple[Image, Image, CameraInfo, float]
        ] = None  # (color, depth, camera_info, monotonic_recv_s)

        # TF2 buffer for gripper→base lookup ------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Match the realsense2_camera driver's RELIABLE QoS (SENSOR_DATA preset is
        # BEST_EFFORT which is incompatible with the driver's publisher).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value

        # Subscribers are routed through a reentrant group so the sync callback can
        # fire concurrently with the service handler (which spends most of its time
        # blocked on the HTTP round trip).
        sub_group = ReentrantCallbackGroup()
        self._color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=sensor_qos, callback_group=sub_group
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=sensor_qos, callback_group=sub_group
        )
        self._info_sub = message_filters.Subscriber(
            self, CameraInfo, info_topic, qos_profile=sensor_qos, callback_group=sub_group
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub, self._info_sub],
            queue_size=self._sync_queue_size,
            slop=self._sync_slop_s,
        )
        self._synchronizer.registerCallback(self._on_synced_frames)

        # Publishers for downstream / RViz -------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._best_pose_pub = self.create_publisher(
            PoseStamped, "~/best_grasp", latched_qos
        )
        self._pose_array_pub = self.create_publisher(
            PoseArray, "~/grasps", latched_qos
        )

        # Debug verification: republish the server's RAW camera-frame poses
        # (no TF applied) alongside the base-frame poses above. With TF active
        # these must overlap exactly in RViz; a mismatch isolates the bug to the
        # client transform rather than the camera mount calibration.
        self._best_pose_cam_pub = self.create_publisher(
            PoseStamped, "~/best_grasp_camera", latched_qos
        )
        self._pose_array_cam_pub = self.create_publisher(
            PoseArray, "~/grasps_camera", latched_qos
        )

        # Broadcast the best grasp as TF frames so RViz's TF display shows the
        # full X/Y/Z axis triad (X=closing, Z=approach), not just a Pose arrow.
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._grasp_tf_lock = threading.Lock()
        self._grasp_tfs: list[tuple[str, str, Pose]] = []
        self.create_timer(
            0.1,
            self._broadcast_grasp_tfs,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Service ---------------------------------------------------------------------
        srv_group = MutuallyExclusiveCallbackGroup()
        self._service = self.create_service(
            RequestGrasp,
            "~/request_grasp",
            self._handle_request_grasp,
            callback_group=srv_group,
        )
        self._upload_service = self.create_service(
            Trigger,
            "~/upload_capture",
            self._handle_upload_capture,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Poll /poll_publish at 2 Hz to pick up results triggered from the Web UI.
        self.create_timer(0.5, self._poll_publish, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f"grasp_pose_client ready: server_url={self._server_url}, "
            f"color={color_topic}, depth={depth_topic}, camera_info={info_topic}, "
            f"sync_slop={self._sync_slop_s:.3f}s"
        )

        if bool(self.get_parameter("probe_health_on_startup").value):
            self._probe_server_health()

    # ------------------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------------------
    def _probe_server_health(self) -> None:
        try:
            payload = get_health(self._server_url, timeout_s=5.0)
        except Exception as exc:
            self.get_logger().warn(
                f"could not reach grasp server at {self._server_url}: {exc}. "
                "Service calls will still be accepted; they will fail until the server is up."
            )
            return
        status = payload.get("status")
        worker_ready = payload.get("worker_ready")
        if status == "ok" and worker_ready:
            self.get_logger().info(
                f"grasp server reachable, worker_pid={payload.get('worker_pid')}"
            )
        else:
            self.get_logger().warn(
                f"grasp server is degraded (status={status}, worker_ready={worker_ready}). "
                "Inspect /health on the server for details."
            )

    def _poll_publish(self) -> None:
        """Timer callback: handle Web UI capture requests and publish triggers."""
        if poll_capture_request(self._server_url):
            self._do_upload_capture()

        result = poll_publish(self._server_url)
        if result is None:
            return

        self.get_logger().info(
            f"poll_publish: received run_id={result.run_id}, {len(result.grasps)} grasp(s)"
        )

        T_base_camera = self._lookup_T_base_camera()
        publish_frame_id = self._robot_base_frame_id if T_base_camera is not None else result.frame_id
        stamp = self.get_clock().now().to_msg()

        grasps_msgs: list[PoseStamped] = []
        scores: list[float] = []
        widths: list[float] = []
        for entry in result.grasps:
            pose_data = (
                self._transform_to_base(entry, T_base_camera, self._grasp_offset_base)
                if T_base_camera is not None
                else entry
            )
            pose_stamped = self._build_pose_stamped(pose_data, stamp=stamp, frame_id=publish_frame_id)
            if pose_stamped is None:
                continue
            grasps_msgs.append(pose_stamped)
            scores.append(float(entry.get("score", 0.0)))
            width_value = entry.get("width_m")
            widths.append(float(width_value) if width_value is not None else float("nan"))

        if not grasps_msgs:
            self.get_logger().warn("poll_publish: no usable grasps in result")
            return

        # Reuse the visualisation publisher so rail_follower gets ~/best_grasp.
        fake_response = type("R", (), {
            "grasps": grasps_msgs,
            "frame_id": publish_frame_id,
        })()
        self._publish_visualisation(fake_response)

        # Debug: raw camera-frame overlay + grasp TF frames for RViz verification.
        cam_best = self._publish_debug_camera(result.grasps, result.frame_id, stamp)
        self._set_grasp_tfs(
            base_pose=grasps_msgs[0].pose,
            base_frame=publish_frame_id,
            cam_pose=cam_best.pose if cam_best is not None else None,
            cam_frame=result.frame_id,
        )

        self.get_logger().info(
            f"poll_publish: published {len(grasps_msgs)} grasp(s) in frame '{publish_frame_id}', "
            f"best score={scores[0]:.4f}"
        )

    def _lookup_T_base_camera(self, stamp=None) -> Optional[np.ndarray]:
        """Look up camera→base transform; stamp=None uses the latest available TF."""
        tf_time = Time.from_msg(stamp) if stamp is not None else Time()
        try:
            ts = self._tf_buffer.lookup_transform(
                self._robot_base_frame_id,
                self._gripper_frame_id,
                tf_time,
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout_s),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF lookup {self._gripper_frame_id}→{self._robot_base_frame_id} failed: {exc}. "
                "base_frame output will be omitted."
            )
            return None

        t = ts.transform.translation
        q = ts.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
            [  2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
            [  2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ], dtype=float)
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z
        return T

    @staticmethod
    def _transform_to_base(
        entry: dict,
        T_base_camera: np.ndarray,
        offset_base: Optional[np.ndarray] = None,
    ) -> dict:
        """Apply T_base_camera to a camera-frame grasp entry, returning base-frame position/quaternion.

        ``offset_base`` (meters, optional) is a constant correction added to the
        resulting base-frame position to absorb a systematic extrinsic bias.
        """
        pose_4x4 = entry.get("pose_4x4")
        if pose_4x4 is not None:
            T_grasp_cam = np.asarray(pose_4x4, dtype=np.float64)
        else:
            position = entry.get("position_xyz", [0.0, 0.0, 0.0])
            quat = entry.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
            x, y, z, w = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            R = np.array([
                [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
                [  2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
                [  2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
            ], dtype=np.float64)
            T_grasp_cam = np.eye(4)
            T_grasp_cam[:3, :3] = R
            T_grasp_cam[0, 3] = float(position[0])
            T_grasp_cam[1, 3] = float(position[1])
            T_grasp_cam[2, 3] = float(position[2])

        T_grasp_base = T_base_camera @ T_grasp_cam
        R_base = T_grasp_base[:3, :3]
        t_base = T_grasp_base[:3, 3]
        if offset_base is not None:
            t_base = t_base + offset_base

        # Rotation matrix → quaternion (Shepperd's method)
        trace = R_base[0, 0] + R_base[1, 1] + R_base[2, 2]
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R_base[2, 1] - R_base[1, 2]) / s
            qy = (R_base[0, 2] - R_base[2, 0]) / s
            qz = (R_base[1, 0] - R_base[0, 1]) / s
        elif (R_base[0, 0] > R_base[1, 1]) and (R_base[0, 0] > R_base[2, 2]):
            s = np.sqrt(1.0 + R_base[0, 0] - R_base[1, 1] - R_base[2, 2]) * 2.0
            qw = (R_base[2, 1] - R_base[1, 2]) / s
            qx = 0.25 * s
            qy = (R_base[0, 1] + R_base[1, 0]) / s
            qz = (R_base[0, 2] + R_base[2, 0]) / s
        elif R_base[1, 1] > R_base[2, 2]:
            s = np.sqrt(1.0 + R_base[1, 1] - R_base[0, 0] - R_base[2, 2]) * 2.0
            qw = (R_base[0, 2] - R_base[2, 0]) / s
            qx = (R_base[0, 1] + R_base[1, 0]) / s
            qy = 0.25 * s
            qz = (R_base[1, 2] + R_base[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R_base[2, 2] - R_base[0, 0] - R_base[1, 1]) * 2.0
            qw = (R_base[1, 0] - R_base[0, 1]) / s
            qx = (R_base[0, 2] + R_base[2, 0]) / s
            qy = (R_base[1, 2] + R_base[2, 1]) / s
            qz = 0.25 * s

        q = np.array([qx, qy, qz, qw])
        norm = float(np.linalg.norm(q))
        if norm > 0.0:
            q = q / norm
        if q[3] < 0.0:
            q = -q

        return {
            "position_xyz": t_base.tolist(),
            "quaternion_xyzw": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        }

    def _on_synced_frames(
        self,
        color_msg: Image,
        depth_msg: Image,
        info_msg: CameraInfo,
    ) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._snapshot_lock:
            self._latest = (color_msg, depth_msg, info_msg, now)

    def _take_snapshot(
        self,
    ) -> tuple[Image, Image, CameraInfo]:
        with self._snapshot_lock:
            snapshot = self._latest

        if snapshot is None:
            raise RuntimeError(
                "no synchronized color+depth+camera_info snapshot yet. "
                "Is realsense2_camera publishing? "
                f"Topics: color/depth/info subscribers received nothing within slop={self._sync_slop_s}s."
            )
        color_msg, depth_msg, info_msg, recv_t = snapshot
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - recv_t
        if age > self._max_snapshot_age_s:
            raise RuntimeError(
                f"latest synchronized snapshot is {age:.2f}s old (> max_snapshot_age_s "
                f"{self._max_snapshot_age_s:.2f}s). Refusing to send stale data."
            )
        return color_msg, depth_msg, info_msg

    def _publish_visualisation(self, response: RequestGrasp.Response) -> None:
        if not response.grasps:
            return
        # Best grasp (also used by downstream planners).
        self._best_pose_pub.publish(response.grasps[0])

        pose_array = PoseArray()
        # All grasps share the same frame_id (server already ensures that).
        pose_array.header = Header(
            stamp=response.grasps[0].header.stamp,
            frame_id=response.frame_id,
        )
        pose_array.poses = [item.pose for item in response.grasps]
        self._pose_array_pub.publish(pose_array)

    def _publish_debug_camera(
        self,
        raw_entries: list[dict],
        camera_frame_id: str,
        stamp,
    ) -> Optional[PoseStamped]:
        """Publish the server's untransformed camera-frame poses for RViz overlap checks.

        Returns the best camera-frame PoseStamped (or None if nothing usable).
        """
        cam_msgs: list[PoseStamped] = []
        for entry in raw_entries:
            ps = self._build_pose_stamped(entry, stamp=stamp, frame_id=camera_frame_id)
            if ps is not None:
                cam_msgs.append(ps)
        if not cam_msgs:
            return None

        self._best_pose_cam_pub.publish(cam_msgs[0])
        pose_array = PoseArray()
        pose_array.header = Header(stamp=cam_msgs[0].header.stamp, frame_id=camera_frame_id)
        pose_array.poses = [m.pose for m in cam_msgs]
        self._pose_array_cam_pub.publish(pose_array)
        return cam_msgs[0]

    def _set_grasp_tfs(
        self,
        *,
        base_pose: Optional[Pose],
        base_frame: str,
        cam_pose: Optional[Pose],
        cam_frame: str,
    ) -> None:
        """Stash the latest best grasp so the timer can keep its TF frames alive."""
        tfs: list[tuple[str, str, Pose]] = []
        if base_pose is not None:
            tfs.append(("grasp_best", base_frame, base_pose))
        if cam_pose is not None:
            tfs.append(("grasp_best_cam", cam_frame, cam_pose))
        with self._grasp_tf_lock:
            self._grasp_tfs = tfs

    def _broadcast_grasp_tfs(self) -> None:
        """Timer callback: re-broadcast the best grasp TF frames with a fresh stamp."""
        with self._grasp_tf_lock:
            tfs = list(self._grasp_tfs)
        if not tfs:
            return
        now = self.get_clock().now().to_msg()
        msgs: list[TransformStamped] = []
        for child_frame, parent_frame, pose in tfs:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = parent_frame
            tf.child_frame_id = child_frame
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation = pose.orientation
            msgs.append(tf)
        self._tf_broadcaster.sendTransform(msgs)

    # ------------------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------------------
    def _do_upload_capture(self) -> bool:
        """Take a snapshot and upload it to the server. Returns True on success."""
        try:
            color_msg, depth_msg, info_msg = self._take_snapshot()
        except RuntimeError as exc:
            self.get_logger().warn(f"upload_capture: {exc}")
            return False
        try:
            rgb_png, _ = color_msg_to_png_bytes(color_msg, self._bridge)
            depth_npy, _ = depth_msg_to_meters_npy_bytes(depth_msg, self._bridge)
            K_json = camera_info_to_K_json(info_msg)
        except ImageConversionError as exc:
            self.get_logger().error(f"upload_capture: image conversion failed: {exc}")
            return False
        frame_id = color_msg.header.frame_id or "camera_color_optical_frame"
        try:
            upload_capture(
                server_url=self._server_url,
                rgb_png_bytes=rgb_png,
                depth_npy_bytes=depth_npy,
                K_json=K_json,
                frame_id=frame_id,
            )
        except GraspServerError as exc:
            self.get_logger().error(f"upload_capture: upload failed: {exc}")
            return False
        self.get_logger().info(f"upload_capture: uploaded frame_id={frame_id}")
        return True

    def _handle_upload_capture(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        response.success = self._do_upload_capture()
        response.message = "capture uploaded" if response.success else "capture failed (see logs)"
        return response

    def _handle_request_grasp(
        self,
        request: RequestGrasp.Request,
        response: RequestGrasp.Response,
    ) -> RequestGrasp.Response:
        response.success = False
        try:
            color_msg, depth_msg, info_msg = self._take_snapshot()
        except RuntimeError as exc:
            response.message = str(exc)
            self.get_logger().warn(response.message)
            return response

        try:
            rgb_png, color_shape = color_msg_to_png_bytes(color_msg, self._bridge)
            depth_npy, depth_shape = depth_msg_to_meters_npy_bytes(depth_msg, self._bridge)
            ensure_shape_match(color_shape, depth_shape)
            K_json = camera_info_to_K_json(info_msg)
        except ImageConversionError as exc:
            response.message = f"image conversion failed: {exc}"
            self.get_logger().error(response.message)
            return response

        frame_id = color_msg.header.frame_id or "camera_color_optical_frame"
        task_spec = request.task_spec.strip()
        if not task_spec:
            response.message = "task_spec must be a non-empty string"
            self.get_logger().warn(response.message)
            return response

        top_k = request.top_k if request.top_k > 0 else self._default_top_k
        num_candidates = (
            request.num_candidates if request.num_candidates > 0 else self._default_num_candidates
        )
        provider = request.provider or self._default_provider or None
        model = request.model or self._default_model or None

        T_base_camera = self._lookup_T_base_camera(color_msg.header.stamp)

        self.get_logger().info(
            f'request_grasp: task_spec="{task_spec}" top_k={top_k} '
            f"num_candidates={num_candidates} frame_id={frame_id} "
            f"base_frame={'yes' if T_base_camera is not None else 'no'}"
        )

        try:
            result = post_grasp(
                server_url=self._server_url,
                rgb_png_bytes=rgb_png,
                depth_npy_bytes=depth_npy,
                K_json=K_json,
                task_spec=task_spec,
                frame_id=frame_id,
                top_k=top_k,
                num_candidates=num_candidates,
                provider=provider,
                model=model,
                timeout_s=self._request_timeout_s,
            )
        except GraspServerError as exc:
            response.message = f"server error: {exc}"
            self.get_logger().error(response.message)
            return response

        stamp = color_msg.header.stamp  # echo the source frame timestamp
        # Transform camera-frame poses to robot base frame when TF is available.
        if T_base_camera is not None:
            publish_frame_id = self._robot_base_frame_id
        else:
            publish_frame_id = result.frame_id

        grasps_msgs: list[PoseStamped] = []
        scores: list[float] = []
        widths: list[float] = []
        for entry in result.grasps:
            pose_data = self._transform_to_base(entry, T_base_camera, self._grasp_offset_base) if T_base_camera is not None else entry
            pose_stamped = self._build_pose_stamped(
                pose_data, stamp=stamp, frame_id=publish_frame_id
            )
            if pose_stamped is None:
                continue
            grasps_msgs.append(pose_stamped)
            scores.append(float(entry.get("score", 0.0)))
            width_value = entry.get("width_m")
            widths.append(float(width_value) if width_value is not None else float("nan"))

        if not grasps_msgs:
            response.message = "server returned zero usable grasps"
            self.get_logger().warn(response.message)
            return response

        response.success = True
        response.message = f"ok, elapsed_ms={result.elapsed_ms}, run_id={result.run_id}"
        response.run_id = result.run_id
        response.frame_id = publish_frame_id
        response.grasps = grasps_msgs
        response.scores = scores
        response.widths = widths

        self._publish_visualisation(response)

        # Debug: raw camera-frame overlay + grasp TF frames for RViz verification.
        cam_best = self._publish_debug_camera(result.grasps, result.frame_id, stamp)
        self._set_grasp_tfs(
            base_pose=response.grasps[0].pose,
            base_frame=publish_frame_id,
            cam_pose=cam_best.pose if cam_best is not None else None,
            cam_frame=result.frame_id,
        )

        self.get_logger().info(
            f"published {len(grasps_msgs)} grasp(s), best score={scores[0]:.4f}, "
            f"width={widths[0]:.4f}m"
        )
        return response

    @staticmethod
    def _build_pose_stamped(
        entry: dict,
        *,
        stamp,
        frame_id: str,
    ) -> Optional[PoseStamped]:
        position = entry.get("position_xyz")
        quat = entry.get("quaternion_xyzw")
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            return None
        if not isinstance(quat, (list, tuple)) or len(quat) != 4:
            return None

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose = pose
        return msg


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspPoseClientNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
