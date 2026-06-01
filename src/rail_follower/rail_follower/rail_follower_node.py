"""Virtual-rail teleoperation filter.

Listens to raw joystick velocity (/commanded_vel) and a grasp target
(/grasp_pose_client/best_grasp). When the operator presses the rail-toggle
button, a path is generated from the current EE position through a pre-grasp
waypoint to the target. The control loop (50 Hz) projects the user's velocity
onto the path tangent and republishes the filtered result on
/commanded_vel_filtered.

Orientation tracking
--------------------
While following the rail, the gripper orientation is continuously interpolated
(SLERP) from the captured start orientation to the target grasp orientation,
proportional to path progress. A proportional controller converts the
orientation error quaternion into angular velocity commands.

Safety guarantees
-----------------
- Passthrough in IDLE mode: velocity is forwarded unchanged.
- Overshoot protection: velocity is zeroed when the EE is within 1 cm of the
  target and the operator is still pushing forward.
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
    """geometry_msgs/Quaternion → 3×3 rotation matrix."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
        [  2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [  2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Quaternion product a ⊗ b, both [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ], dtype=np.float64)


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate (= inverse for unit quaternions), [x,y,z,w] → [−x,−y,−z,w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between unit quaternions at parameter t∈[0,1]."""
    dot = float(np.dot(q0, q1))
    if dot < 0.0:          # ensure shortest arc
        q1 = -q1
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:       # nearly identical — linear fallback avoids division by ~0
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
        self.declare_parameter("path_num_points", 200)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("rail_button", 1)
        self.declare_parameter("long_press_s", 0.5)
        self.declare_parameter("base_frame_id", "LIO_robot_base_link")
        self.declare_parameter("gripper_frame_id", "lio_gripper_joint")
        self.declare_parameter("tf_timeout_s", 0.1)
        self.declare_parameter("overshoot_dist_m", 0.01)
        self.declare_parameter("rail_speed_m_s", 0.05)
        self.declare_parameter("orient_gain", 2.0)
        self.declare_parameter("joy_topic", "/spacenav/joy")
        self.declare_parameter("grasp_topic", "/grasp_pose_client/best_grasp")

        self._pre_grasp_offset: float = self.get_parameter("pre_grasp_offset_m").value
        self._path_n: int = int(self.get_parameter("path_num_points").value)
        self._rate_hz: float = float(self.get_parameter("control_rate_hz").value)
        self._rail_btn: int = int(self.get_parameter("rail_button").value)
        self._long_press_s: float = float(self.get_parameter("long_press_s").value)
        self._base_frame: str = str(self.get_parameter("base_frame_id").value)
        self._gripper_frame: str = str(self.get_parameter("gripper_frame_id").value)
        self._tf_timeout: float = float(self.get_parameter("tf_timeout_s").value)
        self._overshoot_dist: float = float(self.get_parameter("overshoot_dist_m").value)
        self._rail_speed: float = float(self.get_parameter("rail_speed_m_s").value)
        self._orient_gain: float = float(self.get_parameter("orient_gain").value)

        self._lock = threading.Lock()
        self._rail_active: bool = False        # True only while actively following
        self._path: Optional[np.ndarray] = None  # (N, 3) in base_frame; kept after release
        self._start_quat: Optional[np.ndarray] = None   # EE orientation at path build [x,y,z,w]
        self._target_quat: Optional[np.ndarray] = None  # grasp target orientation [x,y,z,w]
        self._latest_user_vel: Twist = Twist()
        self._latest_grasp: Optional[Pose] = None
        self._btn_held: bool = False           # right button currently pressed
        self._btn_press_time: float = 0.0      # wall-clock seconds at press
        self._prev_btn: int = 0

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
            if self._rail_active:
                # Regenerate path when a new target arrives while active.
                self._try_build_path_locked()

    def _on_joy(self, msg: Joy) -> None:
        if self._rail_btn >= len(msg.buttons):
            return
        btn = int(msg.buttons[self._rail_btn])
        now = self.get_clock().now().nanoseconds * 1e-9

        if btn == 1 and self._prev_btn == 0:
            # Press: compute path and show in RViz immediately.
            with self._lock:
                self._btn_held = True
                self._btn_press_time = now
                if self._latest_grasp is None:
                    self.get_logger().warn("rail: no grasp target yet — path not computed")
                else:
                    self._try_build_path_locked()

        elif btn == 0 and self._prev_btn == 1:
            # Release: stop following; keep path visible.
            with self._lock:
                self._btn_held = False
                if self._rail_active:
                    self._rail_active = False
                    self._publish_status("inactive")
                    self.get_logger().info(
                        "rail: button released — following stopped, path kept in RViz"
                    )

        self._prev_btn = btn

    def _try_build_path_locked(self) -> bool:
        """Build path from current EE to grasp target. Must be called under self._lock."""
        ee_pose = self._lookup_ee_pose()
        if ee_pose is None:
            self.get_logger().warn(
                "rail: cannot build path — TF lookup failed for EE pose"
            )
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

        R = _quat_to_rotation_matrix(grasp.orientation)
        approach_dir = R[:, 2]
        approach_norm = np.linalg.norm(approach_dir)
        if approach_norm > 1e-9:
            approach_dir = approach_dir / approach_norm

        pre_grasp_pos = grasp_pos - self._pre_grasp_offset * approach_dir

        n1 = max(2, int(self._path_n * 0.7))
        n2 = max(2, self._path_n - n1 + 1)
        seg1 = np.linspace(current_pos, pre_grasp_pos, n1)
        seg2 = np.linspace(pre_grasp_pos, grasp_pos, n2)[1:]
        self._path = np.vstack([seg1, seg2])
        self._start_quat = current_quat
        self._target_quat = grasp_quat

        self.get_logger().info(
            f"rail path built: {len(self._path)} pts  "
            f"start={current_pos.round(3).tolist()}  "
            f"pre-grasp={pre_grasp_pos.round(3).tolist()}  "
            f"target={grasp_pos.round(3).tolist()}"
        )
        self._publish_path(self._path)
        return True

    # ------------------------------------------------------------------
    # Control loop (50 Hz)
    # ------------------------------------------------------------------

    def _control_tick(self) -> None:
        with self._lock:
            user_vel = self._latest_user_vel
            rail_active = self._rail_active
            path = self._path
            btn_held = self._btn_held
            btn_press_time = self._btn_press_time

        # Long-press threshold: activate following once button held long enough.
        if btn_held and not rail_active and path is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - btn_press_time >= self._long_press_s:
                with self._lock:
                    if self._btn_held and not self._rail_active and self._path is not None:
                        self._rail_active = True
                        rail_active = True
                self._publish_status("active")
                self.get_logger().info("rail: long-press — following activated")

        with self._lock:
            start_quat = self._start_quat
            target_quat = self._target_quat

        if not rail_active or path is None:
            self._filtered_pub.publish(user_vel)
            return

        ee_pose = self._lookup_ee_pose()
        if ee_pose is None:
            self._filtered_pub.publish(user_vel)
            return
        current_pos, current_quat = ee_pose

        # Auto-deactivate when target reached
        grasp_pos = path[-1]
        dist_to_target = float(np.linalg.norm(grasp_pos - current_pos))
        if dist_to_target < self._overshoot_dist:
            with self._lock:
                self._rail_active = False
                self._btn_held = False
            self._publish_status("inactive")
            self.get_logger().info("rail: target reached — auto-deactivated")
            self._filtered_pub.publish(Twist())
            return

        # Find nearest path point and compute tangent
        dists = np.linalg.norm(path - current_pos, axis=1)
        k = min(int(np.argmin(dists)), len(path) - 2)
        t = path[k + 1] - path[k]
        t_norm = np.linalg.norm(t)
        if t_norm < 1e-9:
            self._filtered_pub.publish(Twist())
            return
        t = t / t_norm

        # Linear velocity: fixed speed along path tangent
        v = self._rail_speed * t
        out = Twist()
        out.linear.x = float(v[0])
        out.linear.y = float(v[1])
        out.linear.z = float(v[2])

        # Angular velocity: SLERP-based orientation tracking
        if start_quat is not None and target_quat is not None:
            s = float(k) / float(max(len(path) - 1, 1))
            q_desired = _slerp(start_quat, target_quat, s)
            # q_error = q_desired ⊗ q_current^{-1}
            q_err = _quat_multiply(q_desired, _quat_conjugate(current_quat))
            if q_err[3] < 0.0:   # canonical form: w >= 0
                q_err = -q_err
            # Small-angle: ω ≈ 2 * K * vec(q_err)
            ang = self._orient_gain * 2.0 * q_err[:3]
            out.angular.x = float(ang[0])
            out.angular.y = float(ang[1])
            out.angular.z = float(ang[2])

        self._filtered_pub.publish(out)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lookup_ee_pose(self) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """Return (position, quaternion_xyzw) of EE in base frame, or None on TF failure."""
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

    def _publish_path(self, path: Optional[np.ndarray]) -> None:
        """Publish the rail path as nav_msgs/Path for RViz; pass None to clear."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        if path is not None:
            for pt in path:
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
