"""Fake-camera replay node: publishes a saved capture as realsense2_camera-style topics.

Use this to exercise ``grasp_pose_client_node`` end-to-end without plugging in a
real RealSense. The data source is the same ``camera_data.npy + color_preview.jpg``
layout produced by ``~/rgbd_data/get_photo.py`` and consumed by
``vla-grasp-server/vg_pipeline``.

Published topics (defaults match the standard ``realsense2_camera`` namespace so
the client node works unchanged):

- ``<color_topic>`` (sensor_msgs/Image, encoding ``bgr8``)
- ``<depth_topic>`` (sensor_msgs/Image, encoding ``32FC1`` in meters)
- ``<camera_info_topic>`` (sensor_msgs/CameraInfo)

All three share the same timestamp on each tick so the client's
ApproximateTimeSynchronizer locks immediately.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image


DEFAULT_COLOR_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"


def _load_capture(capture_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (bgr_uint8 HxWx3, depth_float32_meters HxW, K_float64 3x3)."""
    npy_path = capture_dir / "camera_data.npy"
    if not npy_path.is_file():
        raise FileNotFoundError(f"missing {npy_path}")
    data = np.load(npy_path, allow_pickle=True).item()
    if not isinstance(data, dict):
        raise TypeError(f"{npy_path} did not contain a dict")
    if "depth" not in data or "K" not in data:
        raise KeyError(f"{npy_path} must contain at least 'depth' and 'K' keys")

    depth = np.ascontiguousarray(np.asarray(data["depth"], dtype=np.float32))
    K = np.asarray(data["K"], dtype=np.float64).reshape(3, 3)
    if depth.ndim != 2:
        raise ValueError(f"depth must be HxW, got {depth.shape}")

    if "rgb" in data:
        # get_photo.py stores BGR8 raw (matches realsense2_camera's color/image_raw
        # encoding when configured with format=BGR8).
        bgr = np.asarray(data["rgb"], dtype=np.uint8)
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f"rgb in npy must be HxWx3, got {bgr.shape}")
    else:
        # Fall back to decoding color_preview.jpg via OpenCV (also returns BGR).
        import cv2
        color_path = capture_dir / "color_preview.jpg"
        if not color_path.is_file():
            raise FileNotFoundError(
                f"npy has no 'rgb' key and {color_path} is missing"
            )
        bgr = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"cv2.imread failed on {color_path}")

    if bgr.shape[:2] != depth.shape:
        raise ValueError(
            f"rgb shape {bgr.shape[:2]} != depth shape {depth.shape}; "
            "the capture is not internally consistent."
        )
    return bgr, depth, K


class CaptureReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("capture_replay")

        self.declare_parameter("capture_dir", "")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("color_topic", DEFAULT_COLOR_TOPIC)
        self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
        self.declare_parameter("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC)

        capture_dir_str = str(self.get_parameter("capture_dir").value or "").strip()
        if not capture_dir_str:
            raise RuntimeError(
                "capture_dir parameter is required (e.g. "
                "ros2 run grasp_pose_client capture_replay_node "
                "--ros-args -p capture_dir:=$HOME/vla-grasp-server/captures/20260417_120019)"
            )
        capture_dir = Path(capture_dir_str).expanduser().resolve()
        if not capture_dir.is_dir():
            raise FileNotFoundError(f"capture_dir not found: {capture_dir}")

        bgr, depth, K = _load_capture(capture_dir)
        self._bgr = bgr
        self._depth = depth
        self._K = K
        self._height, self._width = int(depth.shape[0]), int(depth.shape[1])
        self._frame_id: str = str(self.get_parameter("frame_id").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz}")

        self._bridge = CvBridge()

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        self._color_pub = self.create_publisher(Image, color_topic, sensor_qos)
        self._depth_pub = self.create_publisher(Image, depth_topic, sensor_qos)
        self._info_pub = self.create_publisher(CameraInfo, info_topic, sensor_qos)

        self._tick_count = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"replaying {capture_dir} at {rate_hz:.1f} Hz "
            f"({self._width}x{self._height}, frame_id={self._frame_id})"
        )
        self.get_logger().info(
            f"  color  -> {color_topic}"
        )
        self.get_logger().info(
            f"  depth  -> {depth_topic} (32FC1, meters)"
        )
        self.get_logger().info(
            f"  info   -> {info_topic}"
        )

    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()

        color_msg = self._bridge.cv2_to_imgmsg(self._bgr, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self._frame_id

        depth_msg = self._bridge.cv2_to_imgmsg(self._depth, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self._frame_id

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self._frame_id
        info_msg.height = self._height
        info_msg.width = self._width
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [float(v) for v in self._K.flatten().tolist()]
        # R = identity, P = [K | 0]: works for unrectified monocular streams.
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fx, fy, cx, cy = self._K[0, 0], self._K[1, 1], self._K[0, 2], self._K[1, 2]
        info_msg.p = [
            float(fx), 0.0, float(cx), 0.0,
            0.0, float(fy), float(cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        self._color_pub.publish(color_msg)
        self._depth_pub.publish(depth_msg)
        self._info_pub.publish(info_msg)
        self._tick_count += 1
        if self._tick_count % 30 == 0:
            self.get_logger().debug(f"replayed {self._tick_count} frames")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[CaptureReplayNode] = None
    try:
        node = CaptureReplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
