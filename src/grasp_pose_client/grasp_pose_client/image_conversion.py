"""Convert ROS2 image messages into the multipart payload the grasp server expects.

The server contract (see ``grasp_server/request_handling.py`` in the vla-grasp-server repo):

- ``rgb``: multipart file, PNG/JPG, H x W x 3 uint8 (PIL decodes to RGB)
- ``depth``: multipart file, ``.npy`` saved via ``numpy.save`` of a float32 H x W array
  in **meters**
- ``K``: form string, JSON-encoded 3x3 intrinsics

This module isolates the heavy ROS-specific decoding so the node logic stays small and
testable.
"""
from __future__ import annotations

import io
import json
from typing import Tuple

import numpy as np

# OpenCV is only used for PNG encoding (cv2.imencode); we keep the import lazy in case
# someone wants to swap it out for Pillow without changing the import surface.
try:
    import cv2
except ImportError as exc:  # pragma: no cover - rely on rosdep python3-opencv
    raise ImportError(
        "OpenCV (python3-opencv) is required for grasp_pose_client.image_conversion"
    ) from exc


_SUPPORTED_RGB_ENCODINGS = {"rgb8", "bgr8", "rgba8", "bgra8"}
_SUPPORTED_DEPTH_ENCODINGS = {"16UC1", "32FC1"}


class ImageConversionError(ValueError):
    """Raised for inputs the grasp server would reject (bad encoding, shape mismatch, ...)."""


def color_msg_to_png_bytes(color_msg, cv_bridge_instance) -> Tuple[bytes, Tuple[int, int]]:
    """Decode a ``sensor_msgs/Image`` color frame and re-encode as PNG bytes.

    Returns (png_bytes, (height, width)).
    """
    encoding = getattr(color_msg, "encoding", "")
    if encoding not in _SUPPORTED_RGB_ENCODINGS:
        raise ImageConversionError(
            f"Unsupported color encoding {encoding!r}; expected one of {sorted(_SUPPORTED_RGB_ENCODINGS)}."
        )

    # Always normalize the in-memory layout to OpenCV's native BGR so cv2.imencode
    # produces a PNG whose decoded RGB matches the original sensor pixels.
    if encoding in {"rgb8", "rgba8"}:
        cv_image = cv_bridge_instance.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
    else:
        cv_image = cv_bridge_instance.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")

    if cv_image.ndim != 3 or cv_image.shape[2] != 3:
        raise ImageConversionError(
            f"Color image must be HxWx3 after conversion, got {cv_image.shape}."
        )

    ok, buf = cv2.imencode(".png", cv_image)
    if not ok:
        raise ImageConversionError("cv2.imencode failed to produce a PNG buffer.")
    return buf.tobytes(), (int(cv_image.shape[0]), int(cv_image.shape[1]))


def color_msg_to_jpeg_bytes(
    color_msg, cv_bridge_instance, *, quality: int = 75
) -> bytes:
    """Encode a color frame as JPEG bytes (for low-latency streaming)."""
    encoding = getattr(color_msg, "encoding", "")
    if encoding not in _SUPPORTED_RGB_ENCODINGS:
        raise ImageConversionError(
            f"Unsupported color encoding {encoding!r}; expected one of {sorted(_SUPPORTED_RGB_ENCODINGS)}."
        )
    cv_image = cv_bridge_instance.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
    ok, buf = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ImageConversionError("cv2.imencode failed to produce a JPEG buffer.")
    return buf.tobytes()


def depth_msg_to_meters_npy_bytes(depth_msg, cv_bridge_instance) -> Tuple[bytes, Tuple[int, int]]:
    """Decode a ``sensor_msgs/Image`` depth frame as float32 meters, serialize as ``.npy`` bytes.

    Supports ``16UC1`` (millimeters; the realsense2_camera ``aligned_depth_to_color``
    default) and ``32FC1`` (already in meters).
    """
    encoding = getattr(depth_msg, "encoding", "")
    if encoding == "16UC1":
        depth_raw = cv_bridge_instance.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        depth_m = depth_raw.astype(np.float32) / 1000.0
    elif encoding == "32FC1":
        depth_m = cv_bridge_instance.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        depth_m = np.ascontiguousarray(depth_m, dtype=np.float32)
    else:
        raise ImageConversionError(
            f"Unsupported depth encoding {encoding!r}; expected one of {sorted(_SUPPORTED_DEPTH_ENCODINGS)}."
        )

    if depth_m.ndim != 2:
        raise ImageConversionError(
            f"Depth image must be HxW, got shape {depth_m.shape}."
        )

    buf = io.BytesIO()
    np.save(buf, depth_m, allow_pickle=False)
    return buf.getvalue(), (int(depth_m.shape[0]), int(depth_m.shape[1]))


def camera_info_to_K_json(camera_info_msg) -> str:
    """Return the 3x3 intrinsic matrix as a JSON string the server can parse."""
    k_flat = list(getattr(camera_info_msg, "k", ()))
    if len(k_flat) != 9:
        raise ImageConversionError(
            f"CameraInfo.k must contain 9 floats (row-major 3x3), got {len(k_flat)}."
        )
    K = np.asarray(k_flat, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(K)):
        raise ImageConversionError("CameraInfo.k contains non-finite entries.")
    if abs(K[2, 2] - 1.0) > 1e-6:
        raise ImageConversionError(
            f"CameraInfo.k[2,2] should be 1.0, got {K[2, 2]:.6f}. "
            "Is the camera actually streaming?"
        )
    W = getattr(camera_info_msg, "width", 0)
    H = getattr(camera_info_msg, "height", 0)
    return json.dumps(K.tolist())


def ensure_shape_match(
    color_shape: Tuple[int, int],
    depth_shape: Tuple[int, int],
) -> None:
    """Mirror the server-side validation early so we don't pay the HTTP round trip."""
    if color_shape != depth_shape:
        raise ImageConversionError(
            f"Color shape {color_shape} != depth shape {depth_shape}. "
            "Did you remember to use realsense2_camera's aligned_depth_to_color topic?"
        )
