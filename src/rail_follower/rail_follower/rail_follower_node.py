"""Virtual-rail teleoperation filter.

Listens to raw joystick velocity (/commanded_vel) and a grasp target
(/grasp_pose_client/best_grasp). When the operator long-presses the rail
button, the EE moves autonomously in two phases:

  Phase 1 — fly to pre-grasp point (target minus offset along approach axis)
  Phase 2 — insert straight into target along the approach axis

Orientation is SLERP-interpolated from the start orientation to the grasp
orientation, proportional to distance traveled vs total path length.

Safety guarantees
-----------------
- Passthrough in IDLE mode: velocity is forwarded unchanged.
- Overshoot protection: velocity is zeroed when the EE is within 1 cm of the
  final target.
- TF lookup failure → passthrough (fail-open, operator retains control).
"""
from __future__ import annotations

import threading
from typing import Optional

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def _quat_to_rotation_matrix(orientation) -> np.ndarray:
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
        [  2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [  2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _quat_to_rotation_matrix_from_array(q: np.ndarray) -> np.ndarray:
    """Expects a normalized [x, y, z, w] quaternion array."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
        [  2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [  2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ], dtype=np.float64)


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta0 = np.arccos(dot)
    theta = theta0 * t
    sin_theta0 = np.sin(theta0)
    s0 = np.cos(theta) - dot * np.sin(theta) / sin_theta0
    s1 = np.sin(theta) / sin_theta0
    return s0 * q0 + s1 * q1


class RailFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("rail_follower")

        self.declare_parameter("pre_grasp_offset_m", 0.15)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("rail_button", 1)
        self.declare_parameter("long_press_s", 0.5)
        self.declare_parameter("base_frame_id", "LIO_robot_base_link")
        self.declare_parameter("gripper_frame_id", "lio_gripper_joint")
        self.declare_parameter("tf_timeout_s", 0.1)
        self.declare_parameter("overshoot_dist_m", 0.01)
        self.declare_parameter("pre_grasp_threshold_m", 0.02)
        self.declare_parameter("rail_speed_m_s", 0.05)
        # Proportional slow-down: within this distance of the current sub-goal the
        # commanded speed ramps down linearly (avoids overshoot / oscillation).
        self.declare_parameter("slowdown_radius_m", 0.08)
        # Speed floor so the EE keeps creeping in instead of asymptotically stalling.
        self.declare_parameter("min_speed_m_s", 0.008)
        # Stall guard: if dist-to-target fails to improve by stall_min_progress_m for
        # stall_timeout_s seconds, auto-deactivate (fail-safe against never reaching
        # the arrival threshold).
        self.declare_parameter("stall_timeout_s", 3.0)
        self.declare_parameter("stall_min_progress_m", 0.003)
        self.declare_parameter("orient_gain", 2.0)
        self.declare_parameter("use_orientation_control", False)
        self.declare_parameter("joy_topic", "/spacenav/joy")
        self.declare_parameter("grasp_topic", "/grasp_pose_client/best_grasp")
        self._pre_grasp_offset: float = self.get_parameter("pre_grasp_offset_m").value
        self._rate_hz: float = float(self.get_parameter("control_rate_hz").value)
        self._rail_btn: int = int(self.get_parameter("rail_button").value)
        self._long_press_s: float = float(self.get_parameter("long_press_s").value)
        self._base_frame: str = str(self.get_parameter("base_frame_id").value)
        self._gripper_frame: str = str(self.get_parameter("gripper_frame_id").value)
        self._tf_timeout: float = float(self.get_parameter("tf_timeout_s").value)
        self._overshoot_dist: float = float(self.get_parameter("overshoot_dist_m").value)
        self._pre_grasp_threshold: float = float(self.get_parameter("pre_grasp_threshold_m").value)
        self._rail_speed: float = float(self.get_parameter("rail_speed_m_s").value)
        self._slowdown_radius: float = float(self.get_parameter("slowdown_radius_m").value)
        self._min_speed: float = float(self.get_parameter("min_speed_m_s").value)
        self._stall_timeout: float = float(self.get_parameter("stall_timeout_s").value)
        self._stall_min_progress: float = float(self.get_parameter("stall_min_progress_m").value)
        self._orient_gain: float = float(self.get_parameter("orient_gain").value)
        self._use_orientation_control: bool = bool(self.get_parameter("use_orientation_control").value)

        self._lock = threading.Lock()
        self._rail_active: bool = False
        self._phase: int = 1                          # 1 = to pre-grasp, 2 = to target
        self._start_pos: Optional[np.ndarray] = None  # EE position when activated
        self._pre_grasp_pos: Optional[np.ndarray] = None
        self._target_pos: Optional[np.ndarray] = None
        self._total_dist: float = 0.0                 # dist(start→pre_grasp) + dist(pre_grasp→target)
        self._phase1_dist: float = 0.0                # dist(start→pre_grasp)
        self._start_quat: Optional[np.ndarray] = None
        self._target_quat: Optional[np.ndarray] = None
        self._phase2_start_quat: Optional[np.ndarray] = None  # sampled at Phase 1→2 transition
        self._phase2_dist: float = 0.0                         # distance from pre_grasp to target
        self._latest_user_vel: Twist = Twist()
        self._latest_grasp: Optional[Pose] = None
        self._btn_held: bool = False
        self._btn_press_time: float = 0.0
        self._prev_btn: int = 0
        # Stall detection: best (smallest) distance-to-target seen so far and the
        # timestamp at which it last improved.
        self._best_dist_to_target: float = float("inf")
        self._last_progress_time: float = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._filtered_pub = self.create_publisher(Twist, "/commanded_vel_filtered", 10)
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._path_pub = self.create_publisher(Path, "~/rail_path", 10)

        self.create_subscription(Twist, "/commanded_vel", self._on_user_vel, 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter("grasp_topic").value,
            self._on_grasp,
            10,
        )
        self.create_subscription(
            Joy,
            self.get_parameter("joy_topic").value,
            self._on_joy,
            10,
        )

        self.create_timer(1.0 / self._rate_hz, self._control_tick)

        self.get_logger().info(
            f"rail_follower ready  base={self._base_frame}  "
            f"gripper={self._gripper_frame}  rail_btn={self._rail_btn}  "
            f"long_press={self._long_press_s}s"
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_user_vel(self, msg: Twist) -> None:
        with self._lock:
            self._latest_user_vel = msg

    def _on_grasp(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_grasp = msg.pose
            # Do not rebuild path while rail is active — path is locked at activation

    def _on_joy(self, msg: Joy) -> None:
        if self._rail_btn >= len(msg.buttons):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        btn = int(msg.buttons[self._rail_btn])
        if btn == 1 and self._prev_btn == 0:
            with self._lock:
                self._btn_held = True
                self._btn_press_time = now
                if self._latest_grasp is None:
                    self.get_logger().warn("rail: no grasp target yet")
                else:
                    self._try_build_path_locked()
        elif btn == 0 and self._prev_btn == 1:
            with self._lock:
                self._btn_held = False
        self._prev_btn = btn

    def _try_build_path_locked(self) -> bool:
        """Compute pre-grasp and target positions. Must be called under self._lock."""
        ee_pose = self._lookup_ee_pose()
        if ee_pose is None:
            self.get_logger().warn("rail: cannot build path — TF lookup failed")
            return False
        if self._latest_grasp is None:
            return False

        current_pos, current_quat = ee_pose
        grasp = self._latest_grasp
        grasp_pos = np.array([grasp.position.x, grasp.position.y, grasp.position.z])
        q = grasp.orientation
        grasp_quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        norm = np.linalg.norm(grasp_quat)
        if norm > 1e-9:
            grasp_quat = grasp_quat / norm

        R = _quat_to_rotation_matrix_from_array(grasp_quat)
        approach_dir = R[:, 2]
        approach_norm = np.linalg.norm(approach_dir)
        if approach_norm > 1e-9:
            approach_dir = approach_dir / approach_norm

        pre_grasp_pos = grasp_pos - self._pre_grasp_offset * approach_dir

        self._start_pos = current_pos.copy()
        self._pre_grasp_pos = pre_grasp_pos
        self._target_pos = grasp_pos
        self._phase = 1
        self._phase1_dist = float(np.linalg.norm(pre_grasp_pos - current_pos))
        self._total_dist = self._phase1_dist + float(np.linalg.norm(grasp_pos - pre_grasp_pos))
        self._start_quat = current_quat
        self._target_quat = grasp_quat
        self._phase2_start_quat = None  # will be sampled at Phase 1→2 transition
        self._phase2_dist = float(np.linalg.norm(grasp_pos - pre_grasp_pos))

        self.get_logger().info(
            f"rail path set  start={current_pos.round(3).tolist()}  "
            f"pre-grasp={pre_grasp_pos.round(3).tolist()}  "
            f"target={grasp_pos.round(3).tolist()}"
        )
        self._publish_path([current_pos, pre_grasp_pos, grasp_pos])
        return True

    # ------------------------------------------------------------------
    # Control loop (50 Hz)
    # ------------------------------------------------------------------

    def _control_tick(self) -> None:
        with self._lock:
            user_vel = self._latest_user_vel
            rail_active = self._rail_active
            btn_held = self._btn_held
            btn_press_time = self._btn_press_time
            target_pos = self._target_pos

        now = self.get_clock().now().nanoseconds * 1e-9

        # Long-press threshold: activate grasp following.
        if btn_held and not rail_active and target_pos is not None:
            if now - btn_press_time >= self._long_press_s:
                with self._lock:
                    if self._btn_held and not self._rail_active and self._target_pos is not None:
                        self._rail_active = True
                        rail_active = True
                        # Arm the stall guard from the moment motion starts.
                        self._best_dist_to_target = float("inf")
                        self._last_progress_time = now
                self._publish_status("active")
                self.get_logger().info("rail: long-press right — grasp following activated")

        with self._lock:
            start_pos = self._start_pos
            pre_grasp_pos = self._pre_grasp_pos
            target_pos = self._target_pos
            phase = self._phase
            total_dist = self._total_dist
            phase1_dist = self._phase1_dist
            start_quat = self._start_quat
            target_quat = self._target_quat
            phase2_start_quat = self._phase2_start_quat
            phase2_dist = self._phase2_dist

        if not rail_active or target_pos is None:
            self._filtered_pub.publish(user_vel)
            return

        ee_pose = self._lookup_ee_pose()
        if ee_pose is None:
            self._filtered_pub.publish(user_vel)
            return
        current_pos, current_quat = ee_pose

        # Auto-deactivate when final target reached
        dist_to_target = float(np.linalg.norm(target_pos - current_pos))
        self.get_logger().debug(f"rail: dist_to_target={dist_to_target:.4f} m  phase={phase}")
        if dist_to_target < self._overshoot_dist:
            with self._lock:
                self._rail_active = False
                self._btn_held = False
            self._publish_status("inactive")
            self.get_logger().info(
                f"rail: target reached ({dist_to_target:.4f} m) — auto-deactivated"
            )
            self._filtered_pub.publish(Twist())
            return

        # Stall guard: track best progress; bail out if it plateaus for too long.
        if dist_to_target < self._best_dist_to_target - self._stall_min_progress:
            self._best_dist_to_target = dist_to_target
            self._last_progress_time = now
        elif now - self._last_progress_time > self._stall_timeout:
            with self._lock:
                self._rail_active = False
                self._btn_held = False
            self._publish_status("inactive")
            self.get_logger().warn(
                f"rail: stalled at {dist_to_target:.4f} m for >{self._stall_timeout:.1f}s "
                "(no progress) — auto-deactivated"
            )
            self._filtered_pub.publish(Twist())
            return

        # Phase transition: pre-grasp reached → switch to insertion phase
        if phase == 1 and pre_grasp_pos is not None:
            if float(np.linalg.norm(pre_grasp_pos - current_pos)) < self._pre_grasp_threshold:
                with self._lock:
                    self._phase = 2
                    self._phase2_start_quat = current_quat.copy()
                phase = 2
                phase2_start_quat = current_quat.copy()
                self.get_logger().info("rail: pre-grasp reached — inserting to target")

        # Direction toward current sub-goal
        goal = target_pos if phase == 2 else pre_grasp_pos
        direction = goal - current_pos
        dist_to_goal = float(np.linalg.norm(direction))
        if dist_to_goal < 1e-9:
            self._filtered_pub.publish(Twist())
            return
        direction = direction / dist_to_goal

        # Proportional slow-down near the sub-goal: ramp speed down linearly inside
        # slowdown_radius, but never below min_speed so the EE keeps closing in.
        speed = self._rail_speed
        if self._slowdown_radius > 1e-6 and dist_to_goal < self._slowdown_radius:
            speed = self._rail_speed * (dist_to_goal / self._slowdown_radius)
            speed = max(speed, self._min_speed)
        speed = min(speed, self._rail_speed)

        out = Twist()
        v = speed * direction
        out.linear.x = float(v[0])
        out.linear.y = float(v[1])
        out.linear.z = float(v[2])

        # Orientation control is disabled by default. When enabled, Phase 2 SLERPs from the
        # orientation sampled at pre-grasp arrival to the target grasp orientation.
        # Disabled by default because unreachable angular commands cause the IK to consume
        # positional DOF, drifting the EE away from the target and preventing termination.
        if self._use_orientation_control and phase == 2 and target_quat is not None \
                and phase2_start_quat is not None and phase2_dist > 1e-9:
            dist_into_phase2 = float(
                np.linalg.norm(current_pos - (pre_grasp_pos if pre_grasp_pos is not None else start_pos))
            )
            s = float(np.clip(dist_into_phase2 / phase2_dist, 0.0, 1.0))
            q_desired = _slerp(phase2_start_quat, target_quat, s)
            q_err = _quat_multiply(q_desired, _quat_conjugate(current_quat))
            if q_err[3] < 0.0:
                q_err = -q_err
            ang = self._orient_gain * q_err[:3]
            out.angular.x = float(ang[0])
            out.angular.y = float(ang[1])
            out.angular.z = float(ang[2])

        self._filtered_pub.publish(out)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lookup_ee_pose(self) -> Optional[tuple[np.ndarray, np.ndarray]]:
        try:
            ts = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._gripper_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF {self._gripper_frame}→{self._base_frame} failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return None
        t = ts.transform.translation
        q = ts.transform.rotation
        pos = np.array([t.x, t.y, t.z], dtype=np.float64)
        quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        return pos, quat

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_path(self, waypoints: list) -> None:
        """Publish waypoints as nav_msgs/Path for RViz."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        for pt in waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = float(pt[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._path_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RailFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
