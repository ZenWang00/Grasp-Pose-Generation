"""Virtual-rail teleoperation filter.

Listens to raw joystick velocity (/commanded_vel) and a grasp target
(/grasp_pose_client/best_grasp). When the operator presses the rail-toggle
button, a path is generated from the current EE position through a pre-grasp
waypoint to the target. The control loop (50 Hz) projects the user's velocity
onto the path tangent and republishes the filtered result on
/commanded_vel_filtered.

Safety guarantees
-----------------
- Passthrough in IDLE mode: velocity is forwarded unchanged.
- Overshoot protection: velocity is zeroed when the EE is within 1 cm of the
  target and the operator is still pushing forward.
- Angular velocity is zeroed while rail mode is active to prevent the gripper
  from rotating into a misaligned approach angle.
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
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def _quat_to_rotation_matrix(orientation) -> np.ndarray:
    """geometry_msgs/Quaternion → 3×3 rotation matrix."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),   2*(x*z + w*y)],
        [  2*(x*y + w*z), 1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [  2*(x*z - w*y),   2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


class RailFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("rail_follower")

        self.declare_parameter("pre_grasp_offset_m", 0.15)
        self.declare_parameter("path_num_points", 200)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("rail_toggle_button", 1)
        self.declare_parameter("base_frame_id", "LIO_robot_base_link")
        self.declare_parameter("gripper_frame_id", "lio_gripper_joint")
        self.declare_parameter("tf_timeout_s", 0.1)
        self.declare_parameter("overshoot_dist_m", 0.01)
        self.declare_parameter("joy_topic", "/spacenav/joy")
        self.declare_parameter("grasp_topic", "/grasp_pose_client/best_grasp")

        self._pre_grasp_offset: float = self.get_parameter("pre_grasp_offset_m").value
        self._path_n: int = int(self.get_parameter("path_num_points").value)
        self._rate_hz: float = float(self.get_parameter("control_rate_hz").value)
        self._toggle_btn: int = int(self.get_parameter("rail_toggle_button").value)
        self._base_frame: str = str(self.get_parameter("base_frame_id").value)
        self._gripper_frame: str = str(self.get_parameter("gripper_frame_id").value)
        self._tf_timeout: float = float(self.get_parameter("tf_timeout_s").value)
        self._overshoot_dist: float = float(self.get_parameter("overshoot_dist_m").value)

        self._lock = threading.Lock()
        self._rail_active: bool = False
        self._path: Optional[np.ndarray] = None       # (N, 3) in base_frame
        self._latest_user_vel: Twist = Twist()
        self._latest_grasp: Optional[Pose] = None
        self._prev_btn: int = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._filtered_pub = self.create_publisher(Twist, "/commanded_vel_filtered", 10)
        self._status_pub = self.create_publisher(String, "~/status", 10)

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
            f"gripper={self._gripper_frame}  toggle_btn={self._toggle_btn}"
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
        if self._toggle_btn >= len(msg.buttons):
            return
        btn = int(msg.buttons[self._toggle_btn])
        if btn == 1 and self._prev_btn == 0:
            self._toggle_rail()
        self._prev_btn = btn

    # ------------------------------------------------------------------
    # Rail toggle
    # ------------------------------------------------------------------

    def _toggle_rail(self) -> None:
        with self._lock:
            if self._rail_active:
                self._rail_active = False
                self._path = None
                self.get_logger().info("rail mode DEACTIVATED — free teleoperation")
                self._publish_status("inactive")
            else:
                if self._latest_grasp is None:
                    self.get_logger().warn(
                        "rail toggle ignored: no grasp target received yet"
                    )
                    return
                built = self._try_build_path_locked()
                if built:
                    self._rail_active = True
                    self.get_logger().info("rail mode ACTIVATED")
                    self._publish_status("active")

    def _try_build_path_locked(self) -> bool:
        """Build path from current EE to grasp target. Must be called under self._lock."""
        current_pos = self._lookup_ee_position()
        if current_pos is None:
            self.get_logger().warn(
                "rail: cannot build path — TF lookup failed for EE position"
            )
            return False
        if self._latest_grasp is None:
            return False

        grasp = self._latest_grasp
        grasp_pos = np.array([grasp.position.x, grasp.position.y, grasp.position.z])

        R = _quat_to_rotation_matrix(grasp.orientation)
        approach_dir = R[:, 2]
        norm = np.linalg.norm(approach_dir)
        if norm > 1e-9:
            approach_dir = approach_dir / norm

        pre_grasp_pos = grasp_pos - self._pre_grasp_offset * approach_dir

        n1 = max(2, int(self._path_n * 0.7))
        n2 = max(2, self._path_n - n1 + 1)
        seg1 = np.linspace(current_pos, pre_grasp_pos, n1)
        seg2 = np.linspace(pre_grasp_pos, grasp_pos, n2)[1:]
        self._path = np.vstack([seg1, seg2])

        self.get_logger().info(
            f"rail path built: {len(self._path)} pts  "
            f"start={current_pos.round(3).tolist()}  "
            f"pre-grasp={pre_grasp_pos.round(3).tolist()}  "
            f"target={grasp_pos.round(3).tolist()}"
        )
        return True

    # ------------------------------------------------------------------
    # Control loop (50 Hz)
    # ------------------------------------------------------------------

    def _control_tick(self) -> None:
        with self._lock:
            user_vel = self._latest_user_vel
            rail_active = self._rail_active
            path = self._path

        if not rail_active or path is None:
            self._filtered_pub.publish(user_vel)
            return

        current_pos = self._lookup_ee_position()
        if current_pos is None:
            # TF unavailable → fail-open, pass raw velocity through
            self._filtered_pub.publish(user_vel)
            return

        # Find nearest path point
        dists = np.linalg.norm(path - current_pos, axis=1)
        k = int(np.argmin(dists))
        k = min(k, len(path) - 2)

        # Tangent direction
        t = path[k + 1] - path[k]
        t_norm = np.linalg.norm(t)
        if t_norm < 1e-9:
            self._filtered_pub.publish(user_vel)
            return
        t = t / t_norm

        # Project user velocity onto tangent
        v_user = np.array([user_vel.linear.x, user_vel.linear.y, user_vel.linear.z])
        projection = float(np.dot(v_user, t))
        v_robot = projection * t

        # Overshoot protection: freeze when close to target and still pushing forward
        grasp_pos = path[-1]
        dist_to_target = float(np.linalg.norm(grasp_pos - current_pos))
        if dist_to_target < self._overshoot_dist and projection > 0:
            v_robot = np.zeros(3)

        out = Twist()
        out.linear.x = float(v_robot[0])
        out.linear.y = float(v_robot[1])
        out.linear.z = float(v_robot[2])
        # Angular velocity zeroed: prevents gripper rotation misaligning the approach axis
        self._filtered_pub.publish(out)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lookup_ee_position(self) -> Optional[np.ndarray]:
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
        return np.array([t.x, t.y, t.z], dtype=np.float64)

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


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
