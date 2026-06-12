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
from sensor_msgs.msg import CameraInfo, Image, JointState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger

import message_filters
from cv_bridge import CvBridge

from grasp_pose_client_msgs.srv import RequestGrasp

from .grasp_logger import GraspLogger
from .http_client import GraspServerError, get_health, poll_capture_request, poll_publish, post_grasp, submit_ik_result, upload_capture
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

# Maps the grasp axis convention (X=closing, Y=lateral, Z=approach) onto the LIO
# TCP convention of lio_tcp_link (X=approach, Y=closing, Z=lateral), so published
# quaternions can be consumed directly as the lio_tcp_link target orientation:
# R_tcp = R_grasp @ GRASP_TO_TCP_AXES  (columns: TCP X←grasp Z, Y←X, Z←Y).
GRASP_TO_TCP_AXES = np.array([
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
])


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
        self.declare_parameter("robot_base_frame_id", "LIO_base_link")
        self.declare_parameter("tf_timeout_s", 0.2)
        # Constant correction (meters) added to every grasp position AFTER it has been
        # transformed into the robot base frame. Use this to absorb a systematic
        # hand-eye / extrinsic bias. Axes follow LIO_base_link (the IK URDF root):
        #   +x = forward (arm extend direction), +y = left, +z = up.
        self.declare_parameter("grasp_offset_base_xyz", [0.0, 0.0, 0.0])
        # Constant correction expressed in the FINAL TCP frame (after the grasp→TCP
        # axis remap): +x = approach (deeper into the grasp), +y = closing axis
        # (between the fingers), +z = lateral. Rotates with the commanded grasp
        # orientation, so use it for tool-side biases (hand-eye lateral error,
        # asymmetric passive finger, grasp depth preference).
        self.declare_parameter("grasp_offset_tool_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ik_urdf_path", "")
        self.declare_parameter("ik_base_link", "LIO_base_link")
        self.declare_parameter("ik_tip_link", "lio_tcp_link")
        self.declare_parameter("ik_max_iter", 200)
        self.declare_parameter("ik_eps", 1e-4)
        self.declare_parameter("ik_dt", 0.1)
        self.declare_parameter("ik_damp", 1e-6)
        self.declare_parameter("ik_bypass", True)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("log_dir", "~/grasp_logs")
        self.declare_parameter("arm_stop_log_delay_s", 40.0)  # log TCP pose this long after commanded_pose

        self._server_url: str = self.get_parameter("server_url").value
        self._seen_phases: set[str] = set()  # f"{trace_id}:{mode}" already processed
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
        _off_tool = list(self.get_parameter("grasp_offset_tool_xyz").value or [0.0, 0.0, 0.0])
        if len(_off_tool) != 3:
            self.get_logger().warn(
                f"grasp_offset_tool_xyz must have 3 elements, got {_off_tool}; ignoring."
            )
            _off_tool = [0.0, 0.0, 0.0]
        self._grasp_offset_tool: np.ndarray = np.array(_off_tool, dtype=np.float64)
        self._ik_base_link: str = str(self.get_parameter("ik_base_link").value)
        self._ik_tip_link: str = str(self.get_parameter("ik_tip_link").value)
        self._ik_max_iter: int = int(self.get_parameter("ik_max_iter").value)
        self._ik_eps: float = float(self.get_parameter("ik_eps").value)
        self._ik_dt: float = float(self.get_parameter("ik_dt").value)
        self._ik_damp: float = float(self.get_parameter("ik_damp").value)
        self._joint_positions: dict[str, float] = {}
        self._pin_model = None
        self._pin_data = None
        self._pin_ee_id: int = -1

        # Internal state --------------------------------------------------------------
        self._bridge = CvBridge()
        self._snapshot_lock = threading.Lock()
        self._latest: Optional[
            tuple[Image, Image, CameraInfo, float]
        ] = None  # (color, depth, camera_info, monotonic_recv_s)

        # TF2 buffer for gripper→base lookup ------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Per-run JSONL logger ---------------------------------------------------------
        _log_dir = str(self.get_parameter("log_dir").value)
        self._grasp_log = GraspLogger(_log_dir)
        self._grasp_log.write("session_start", node="grasp_pose_client",
                               server_url=self._server_url,
                               grasp_offset_base_xyz=list(self._grasp_offset_base),
                               grasp_offset_tool_xyz=list(self._grasp_offset_tool))
        self.get_logger().info(f"Grasp logger active: {self._grasp_log.path}")

        # Delayed arm-stop snapshot: record the TCP pose a fixed time after each
        # commanded_pose, long enough for the motion to have finished.
        self._arm_stop_log_delay_s: float = float(
            self.get_parameter("arm_stop_log_delay_s").value)
        self._arm_stop_due_time: Optional[float] = None  # monotonic deadline, None = idle
        self._arm_stop_run_id: str = ""

        # JointState subscription for IK seed + Pinocchio IK solver init --------------
        _joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._joint_state_sub = self.create_subscription(
            JointState, _joint_states_topic, self._joint_state_cb, 10
        )
        # Real robot joints straight from the platform — used to cross-check the
        # arm_stopped TF snapshot with our own FK, so a second (stale/mismatched)
        # TF broadcaster can never silently fake a perfect log entry.
        self._real_joint_positions: dict[str, float] = {}
        self._real_joint_state_sub = self.create_subscription(
            JointState, "/lio_joint_states", self._real_joint_state_cb, 10
        )
        self._init_ik_solver()

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
        self._commanded_pose_pub = self.create_publisher(
            PoseStamped, "/commanded_pose", 10
        )
        self._pose_array_pub = self.create_publisher(
            PoseArray, "~/grasps", latched_qos
        )

        # Debug verification: republish the server's RAW camera-frame poses
        # (no TF applied, grasp axis convention) alongside the base-frame poses
        # above (TCP axis convention). With TF active their positions must
        # overlap exactly in RViz, while orientations differ by the fixed
        # grasp→TCP axis permutation. A position mismatch isolates the bug to
        # the client transform rather than the camera mount calibration.
        self._best_pose_cam_pub = self.create_publisher(
            PoseStamped, "~/best_grasp_camera", latched_qos
        )
        self._pose_array_cam_pub = self.create_publisher(
            PoseArray, "~/grasps_camera", latched_qos
        )

        # Broadcast the best grasp as TF frames so RViz's TF display shows the
        # full X/Y/Z axis triad, not just a Pose arrow. grasp_best uses the TCP
        # convention (X=approach, Y=closing, Z=lateral) and should coincide with
        # lio_tcp_link once the arm reaches the target; grasp_best_cam keeps the
        # raw grasp convention (X=closing, Z=approach).
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._grasp_tf_lock = threading.Lock()
        self._grasp_tfs: list[tuple[str, str, Pose]] = []
        self.create_timer(
            0.1,
            self._broadcast_grasp_tfs,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Arm-stop snapshot: 1 Hz check for a pending delayed TCP-pose log
        self.create_timer(
            1.0,
            self._arm_stop_poll,
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

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = float(pos)

    def _real_joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._real_joint_positions[name] = float(pos)

    def _fk_real_tcp(self) -> Optional[np.ndarray]:
        """TCP position in LIO_base_link from the REAL robot joints (/lio_joint_states),
        computed with our own Pinocchio model — independent of the TF tree."""
        if self._pin_model is None or not self._real_joint_positions:
            return None
        import pinocchio as pin
        q = pin.neutral(self._pin_model)
        matched = 0
        for jid in range(1, self._pin_model.njoints):
            joint = self._pin_model.joints[jid]
            if joint.nq != 1:
                continue
            jname = self._pin_model.names[jid]
            if jname in self._real_joint_positions:
                q[joint.idx_q] = self._real_joint_positions[jname]
                matched += 1
        if matched < 6:
            return None
        pin.forwardKinematics(self._pin_model, self._pin_data, q)
        pin.updateFramePlacements(self._pin_model, self._pin_data)
        return np.array(self._pin_data.oMf[self._pin_ee_id].translation)

    def _arm_stop_poll(self) -> None:
        """1 Hz timer: once arm_stop_log_delay_s has elapsed since the last
        commanded_pose, log the current TCP pose as arm_stopped (one-shot).
        On TF failure the snapshot is retried on the next tick."""
        if self._arm_stop_due_time is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._arm_stop_due_time:
            return
        T = self._lookup_tf_matrix(self._robot_base_frame_id, self._ik_tip_link)
        if T is None:
            return
        self._arm_stop_due_time = None
        self._log_arm_stop(T)

    def _log_arm_stop(self, T: np.ndarray) -> None:
        """Write an arm_stopped log entry from the given TCP TF matrix.

        Also records ``position_fk_real``: the TCP position recomputed from the
        REAL robot joints with our own Pinocchio model. If it disagrees with the
        TF value, a second TF broadcaster is polluting the tree and the TF-based
        snapshot must not be trusted.
        """
        t = T[:3, 3]
        q = self._rot_to_quat_xyzw(T[:3, :3])
        extra = {}
        p_fk = self._fk_real_tcp()
        if p_fk is not None:
            extra["position_fk_real"] = {
                "x": float(p_fk[0]), "y": float(p_fk[1]), "z": float(p_fk[2])
            }
            mismatch = float(np.linalg.norm(p_fk - t))
            extra["tf_vs_fk_real_m"] = mismatch
            if mismatch > 0.005:
                self.get_logger().warn(
                    f"arm_stopped: TF tcp and FK(real joints) disagree by "
                    f"{mismatch*1000:.1f} mm — TF tree may have a second broadcaster."
                )
        self._grasp_log.write(
            "arm_stopped",
            run_id=self._arm_stop_run_id,
            frame=self._robot_base_frame_id,
            position={"x": float(t[0]), "y": float(t[1]), "z": float(t[2])},
            orientation={"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
            **extra,
        )

    @staticmethod
    def _rot_to_quat_xyzw(R: np.ndarray) -> list:
        """Rotation matrix → [qx, qy, qz, qw] via Shepperd's method, w ≥ 0."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        q = np.array([qx, qy, qz, qw])
        norm = float(np.linalg.norm(q))
        if norm > 0.0:
            q = q / norm
        if q[3] < 0.0:
            q = -q
        return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    def _init_ik_solver(self) -> None:
        urdf_path = str(self.get_parameter("ik_urdf_path").value)
        if not urdf_path:
            self.get_logger().info("ik_urdf_path not set — IK feasibility check disabled")
            return
        try:
            import pinocchio as pin
            self._pin_model = pin.buildModelFromUrdf(urdf_path)
            self._pin_data = self._pin_model.createData()
            self._pin_ee_id = self._pin_model.getFrameId(self._ik_tip_link)
            if self._pin_ee_id >= self._pin_model.nframes:
                raise ValueError(f"tip link '{self._ik_tip_link}' not found in URDF")
            self.get_logger().info(
                f"Pinocchio IK ready: {self._ik_base_link}→{self._ik_tip_link} "
                f"(nq={self._pin_model.nq}, frame_id={self._pin_ee_id})"
            )
        except ImportError:
            self.get_logger().error(
                "pinocchio not importable — install ros-jazzy-pinocchio and source setup.bash"
            )
        except Exception as exc:
            self.get_logger().error(f"Pinocchio IK solver init failed: {exc}")

    def _pin_solve_ik(
        self, target_SE3, q0: np.ndarray
    ) -> tuple[Optional[np.ndarray], str]:
        """Two-stage Gauss-Newton IK.

        Stage 1: position-only IK (LOCAL_WORLD_ALIGNED, 3-DOF error) to drive the
        end-effector near the target position. This is robust to large initial
        orientation errors (w≈0 quaternions produce log6 vectors with norm≈π that
        cause the single-stage solver to overshoot and diverge).

        Stage 2: full 6-DOF IK starting from the Stage-1 solution, which is now
        in a much smaller basin and converges reliably.

        Returns:
            (q_solution, diag) where diag is a short diagnostic string.
            q_solution is None on failure.
        """
        import pinocchio as pin
        model, data = self._pin_model, self._pin_data

        # ── Stage 1: position only ────────────────────────────────────────────
        q = q0.copy()
        t_tgt = target_SE3.translation
        pos_err = float("inf")
        for _ in range(self._ik_max_iter):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            err3 = t_tgt - data.oMf[self._pin_ee_id].translation
            pos_err = float(np.linalg.norm(err3))
            if pos_err < 1e-3:
                break
            J3 = pin.computeFrameJacobian(
                model, data, q, self._pin_ee_id, pin.LOCAL_WORLD_ALIGNED)[:3]
            v = J3.T @ np.linalg.solve(J3 @ J3.T + self._ik_damp * np.eye(3), err3)
            q = pin.integrate(model, q, v * self._ik_dt)
            q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

        if pos_err >= 1e-3:
            return None, f"stage1_failed: pos_err={pos_err*1000:.2f}mm (position unreachable)"

        # ── Stage 2: full 6-DOF IK from the position-warm seed ───────────────
        err6_norm = float("inf")
        for _ in range(self._ik_max_iter):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            err = pin.log6(data.oMf[self._pin_ee_id].inverse() * target_SE3).vector
            err6_norm = float(np.linalg.norm(err))
            if err6_norm < self._ik_eps:
                return q, "ok"
            J = pin.computeFrameJacobian(model, data, q, self._pin_ee_id, pin.LOCAL)
            v = J.T @ np.linalg.solve(J @ J.T + self._ik_damp * np.eye(6), err)
            q = pin.integrate(model, q, v * self._ik_dt)
            q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

        # Report final residuals broken into position and rotation components
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        final_pos_err = float(np.linalg.norm(
            target_SE3.translation - data.oMf[self._pin_ee_id].translation))
        return None, (
            f"stage2_failed: log6_err={err6_norm:.4f} "
            f"pos_err={final_pos_err*1000:.2f}mm "
            f"(orientation unsolvable after position converged)"
        )

    def _check_ik_feasibility(self, grasps: list[dict]) -> list[dict]:
        if bool(self.get_parameter("ik_bypass").value):
            self.get_logger().warn(
                f"ik_bypass=True — skipping IK check, passing all {len(grasps)} candidate(s) through"
            )
            return grasps

        if self._pin_model is None:
            self.get_logger().warn(
                "IK solver not ready — passing all candidates through",
                throttle_duration_sec=10.0,
            )
            return grasps

        # _gripper_frame_id is the camera optical frame (source of server grasps),
        # same convention as _lookup_T_base_camera(). T = T_{LIO_base_link←camera}.
        T = self._lookup_tf_matrix(self._ik_base_link, self._gripper_frame_id)
        if T is None:
            self.get_logger().warn(
                f"TF {self._gripper_frame_id}→{self._ik_base_link} unavailable "
                "— passing all candidates through",
                throttle_duration_sec=5.0,
            )
            return grasps

        import pinocchio as pin
        q0 = pin.neutral(self._pin_model)
        if not self._joint_positions:
            self.get_logger().warn(
                "No /joint_states received yet — IK seed is all-zeros; "
                "reachability result may be inaccurate",
                throttle_duration_sec=5.0,
            )
        else:
            matched, total = 0, len(self._pin_model.names) - 1
            for jname in self._pin_model.names[1:]:  # skip universe joint
                jid = self._pin_model.getJointId(jname)
                idx = self._pin_model.joints[jid].idx_q
                if jname in self._joint_positions:
                    q0[idx] = self._joint_positions[jname]
                    matched += 1
            if matched == 0:
                self.get_logger().warn(
                    f"No Pinocchio joint names matched /joint_states "
                    f"(model joints: {list(self._pin_model.names[1:])}, "
                    f"received: {list(self._joint_positions.keys())}) "
                    "— IK seed is all-zeros"
                )
            else:
                self.get_logger().debug(f"IK seed: {matched}/{total} joints matched from /joint_states")

        q_neutral = pin.neutral(self._pin_model)
        # Seeds to try in order: joint-state seed first, neutral as fallback.
        seeds = [q0, q_neutral] if np.any(q0 != q_neutral) else [q0]

        passed: list[dict] = []
        for i, grasp in enumerate(grasps):
            p_ik = self._transform_to_base(grasp, T)
            pos = np.array(p_ik["position_xyz"])
            q_xyzw = p_ik["quaternion_xyzw"]
            # Pinocchio Quaternion takes (w, x, y, z); our dict stores (x, y, z, w)
            target_SE3 = pin.SE3(
                pin.Quaternion(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]).toRotationMatrix(),
                pos,
            )
            pose_str = (
                f"pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] "
                f"quat(xyzw)=[{q_xyzw[0]:.3f},{q_xyzw[1]:.3f},{q_xyzw[2]:.3f},{q_xyzw[3]:.3f}]"
            )
            q_sol = None
            seed_diags: list[str] = []
            for j, s in enumerate(seeds):
                sol, diag = self._pin_solve_ik(target_SE3, s)
                seed_diags.append(f"seed{j}:{diag}")
                if sol is not None:
                    q_sol = sol
                    break
            ok = q_sol is not None
            if ok:
                self.get_logger().info(f"IK candidate {i}: {pose_str} → PASS [{seed_diags[0]}]")
                passed.append(grasp)
            else:
                self.get_logger().warn(
                    f"IK candidate {i}: {pose_str} → FAIL "
                    f"[{'; '.join(seed_diags)}]"
                )

        self.get_logger().info(
            f"IK feasibility: {len(passed)}/{len(grasps)} candidates passed"
        )
        return passed

    def _poll_publish(self) -> None:
        """Timer callback: handle Web UI capture requests and publish triggers."""
        if poll_capture_request(self._server_url):
            self._do_upload_capture()

        result = poll_publish(self._server_url)
        if result is None:
            return

        phase_key = f"{result.trace_id}:{result.mode}"
        if phase_key in self._seen_phases:
            return
        self._seen_phases.add(phase_key)

        if result.mode == "ik_check":
            passed = self._check_ik_feasibility(result.grasps)
            if not passed:
                self.get_logger().error(
                    f"[trace={result.trace_id}] all candidates failed IK — submitting empty result"
                )
            try:
                submit_ik_result(self._server_url, result.run_id, result.trace_id, passed)
                self.get_logger().info(
                    f"[trace={result.trace_id}] IK round-trip: "
                    f"submitted {len(passed)}/{len(result.grasps)} candidates"
                )
            except Exception as exc:
                self.get_logger().error(f"[trace={result.trace_id}] submit_ik_result failed: {exc}")
            return  # do NOT publish yet; wait for mode="execute" payload

        # mode == "execute": fall through to existing publish + TF logic
        self.get_logger().info(
            f"[trace={result.trace_id}] poll_publish: received run_id={result.run_id}, "
            f"{len(result.grasps)} grasp(s) for execution"
        )

        # Log best grasp as received from server (camera frame, before any TF)
        if result.grasps:
            g0 = result.grasps[0]
            self._grasp_log.write(
                "server_best_grasp6d",
                run_id=result.run_id,
                score=g0.get("score"),
                frame=result.frame_id,
                position_xyz=g0.get("position_xyz"),
                quaternion_xyzw=g0.get("quaternion_xyzw"),
                pose_4x4=g0.get("pose_4x4"),
            )

        T_base_camera = self._lookup_T_base_camera()
        if T_base_camera is None:
            # Never command a camera-frame pose; drop the seen-marker so the
            # payload is retried on the next poll once TF recovers.
            self._seen_phases.discard(phase_key)
            self.get_logger().error(
                f"[trace={result.trace_id}] TF {self._gripper_frame_id}→"
                f"{self._robot_base_frame_id} unavailable — skipping publish, will retry"
            )
            return
        publish_frame_id = self._robot_base_frame_id
        stamp = self.get_clock().now().to_msg()

        grasps_msgs: list[PoseStamped] = []
        scores: list[float] = []
        widths: list[float] = []
        for entry in result.grasps:
            pose_data = self._transform_to_base(entry, T_base_camera, self._grasp_offset_base, self._grasp_offset_tool)
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

        # Publish best grasp for RViz and forward to /commanded_pose for execution.
        fake_response = type("R", (), {
            "grasps": grasps_msgs,
            "frame_id": publish_frame_id,
        })()
        self._publish_visualisation(fake_response)

        # Log the pose actually sent for execution (base frame, TCP axis convention)
        _p = grasps_msgs[0].pose.position
        _o = grasps_msgs[0].pose.orientation
        self._grasp_log.write(
            "commanded_pose",
            run_id=result.run_id,
            frame=publish_frame_id,
            position={"x": _p.x, "y": _p.y, "z": _p.z},
            orientation={"x": _o.x, "y": _o.y, "z": _o.z, "w": _o.w},
        )
        # Schedule the delayed arm_stopped snapshot for this run
        self._arm_stop_due_time = (
            self.get_clock().now().nanoseconds * 1e-9 + self._arm_stop_log_delay_s
        )
        self._arm_stop_run_id = result.run_id

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

    def _lookup_tf_matrix(self, target_frame: str, source_frame: str) -> Optional[np.ndarray]:
        """Return 4x4 T_{target←source} using the latest TF, or None on failure."""
        try:
            ts = self._tf_buffer.lookup_transform(
                target_frame, source_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout_s),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF {source_frame}→{target_frame} failed: {exc}",
                throttle_duration_sec=5.0,
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
        T[:3, 3] = [t.x, t.y, t.z]
        return T

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
        offset_tool: Optional[np.ndarray] = None,
    ) -> dict:
        """Apply T_base_camera to a camera-frame grasp entry, returning base-frame position/quaternion.

        ``offset_base`` (meters, optional) is a constant correction added to the
        resulting base-frame position to absorb a systematic extrinsic bias.
        ``offset_tool`` (meters, optional) is a constant correction expressed in
        the final TCP frame (x=approach, y=closing, z=lateral); it rotates with
        the commanded grasp orientation.
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
        R_base = T_grasp_base[:3, :3] @ GRASP_TO_TCP_AXES
        t_base = T_grasp_base[:3, 3]
        if offset_tool is not None:
            t_base = t_base + R_base @ offset_tool
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
        best = response.grasps[0]
        self._best_pose_pub.publish(best)
        if best.header.frame_id == self._robot_base_frame_id:
            self._commanded_pose_pub.publish(best)
        else:
            self.get_logger().warn(
                f"best grasp is in frame '{best.header.frame_id}', not base frame "
                f"'{self._robot_base_frame_id}' — not publishing to /commanded_pose"
            )

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
        """Stash the latest best grasp so the timer can keep its TF frames alive.

        Only the base-frame pose is kept alive: the camera rides on the arm, so a
        continuously re-stamped camera-frame TF drifts away with the gripper once
        the arm moves and looks like a target the gripper "never reaches" in RViz.
        The camera-frame pose is still published once on /best_grasp_pose_cam.
        """
        del cam_pose, cam_frame  # intentionally not broadcast as TF
        tfs: list[tuple[str, str, Pose]] = []
        if base_pose is not None:
            tfs.append(("grasp_best", base_frame, base_pose))
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

        # Log best grasp as received from server (camera frame, before any TF)
        if result.grasps:
            g0 = result.grasps[0]
            self._grasp_log.write(
                "server_best_grasp6d",
                run_id=result.run_id,
                score=g0.get("score"),
                frame=result.frame_id,
                position_xyz=g0.get("position_xyz"),
                quaternion_xyzw=g0.get("quaternion_xyzw"),
                pose_4x4=g0.get("pose_4x4"),
            )

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
            pose_data = self._transform_to_base(entry, T_base_camera, self._grasp_offset_base, self._grasp_offset_tool) if T_base_camera is not None else entry
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

        # Log the published best pose (frame indicates whether it reached /commanded_pose)
        _p = response.grasps[0].pose.position
        _o = response.grasps[0].pose.orientation
        self._grasp_log.write(
            "commanded_pose",
            run_id=result.run_id,
            frame=publish_frame_id,
            position={"x": _p.x, "y": _p.y, "z": _p.z},
            orientation={"x": _o.x, "y": _o.y, "z": _o.z, "w": _o.w},
        )
        # Schedule the delayed arm_stopped snapshot only if the pose actually went
        # to /commanded_pose (camera-frame fallback poses are visualisation-only).
        if publish_frame_id == self._robot_base_frame_id:
            self._arm_stop_due_time = (
                self.get_clock().now().nanoseconds * 1e-9 + self._arm_stop_log_delay_s
            )
            self._arm_stop_run_id = result.run_id

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
