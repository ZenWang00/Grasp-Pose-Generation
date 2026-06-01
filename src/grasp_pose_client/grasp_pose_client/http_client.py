"""Thin wrapper around ``requests`` for the VLA grasp server's ``POST /grasp`` endpoint.

Keeping the HTTP layer isolated from the ROS2 node makes it trivial to unit-test against
a recorded server response and to swap transports later.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import requests


class GraspServerError(RuntimeError):
    """Raised when the grasp server returns a non-2xx status or a malformed body."""

    def __init__(self, message: str, *, status_code: int | None = None, body: Any = None) -> None:
        super().__init__(message)
        self.status_code = status_code
        self.body = body


@dataclass
class GraspResult:
    success: bool
    run_id: str
    run_dir: str
    frame_id: str
    elapsed_ms: int
    grasps: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_json(cls, payload: dict[str, Any]) -> "GraspResult":
        grasps = payload.get("grasps") or []
        if not isinstance(grasps, list):
            raise GraspServerError(
                f"Server response 'grasps' must be a list, got {type(grasps).__name__}",
                body=payload,
            )
        return cls(
            success=True,
            run_id=str(payload.get("run_id", "")),
            run_dir=str(payload.get("run_dir", "")),
            frame_id=str(payload.get("frame_id", "")),
            elapsed_ms=int(payload.get("elapsed_ms", 0) or 0),
            grasps=grasps,
        )


def post_grasp(
    *,
    server_url: str,
    rgb_png_bytes: bytes,
    depth_npy_bytes: bytes,
    K_json: str,
    task_spec: str,
    frame_id: str,
    top_k: int,
    num_candidates: int,
    provider: str | None = None,
    model: str | None = None,
    timeout_s: float = 60.0,
) -> GraspResult:
    """POST a single multipart request and return the parsed result."""
    url = server_url.rstrip("/") + "/grasp"
    files = {
        "rgb": ("rgb.png", rgb_png_bytes, "image/png"),
        "depth": ("depth.npy", depth_npy_bytes, "application/octet-stream"),
    }
    data: dict[str, Any] = {
        "K": K_json,
        "task_spec": task_spec,
        "frame_id": frame_id,
        "top_k": str(int(top_k)),
        "num_candidates": str(int(num_candidates)),
    }
    if provider:
        data["provider"] = provider
    if model:
        data["model"] = model

    try:
        response = requests.post(url, files=files, data=data, timeout=timeout_s)
    except requests.RequestException as exc:
        raise GraspServerError(f"HTTP request failed: {exc}") from exc

    if response.status_code != 200:
        snippet = response.text[:500] if response.text else ""
        try:
            body = response.json()
        except Exception:
            body = snippet
        raise GraspServerError(
            f"server returned HTTP {response.status_code}: {snippet}",
            status_code=response.status_code,
            body=body,
        )

    try:
        payload = response.json()
    except ValueError as exc:
        raise GraspServerError(f"server response was not JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise GraspServerError(
            f"server response root must be an object, got {type(payload).__name__}",
            body=payload,
        )
    return GraspResult.from_json(payload)


def upload_capture(
    *,
    server_url: str,
    rgb_png_bytes: bytes,
    depth_npy_bytes: bytes,
    K_json: str,
    frame_id: str,
    timeout_s: float = 10.0,
) -> dict[str, Any]:
    """POST rgb+depth+K to /upload_capture; returns the server JSON payload."""
    url = server_url.rstrip("/") + "/upload_capture"
    files = {
        "rgb": ("rgb.png", rgb_png_bytes, "image/png"),
        "depth": ("depth.npy", depth_npy_bytes, "application/octet-stream"),
    }
    data: dict[str, Any] = {"K": K_json, "frame_id": frame_id}
    try:
        response = requests.post(url, files=files, data=data, timeout=timeout_s)
    except requests.RequestException as exc:
        raise GraspServerError(f"HTTP request failed: {exc}") from exc
    if response.status_code != 200:
        snippet = response.text[:500] if response.text else ""
        try:
            body = response.json()
        except Exception:
            body = snippet
        raise GraspServerError(
            f"server returned HTTP {response.status_code}: {snippet}",
            status_code=response.status_code,
            body=body,
        )
    try:
        return response.json()
    except ValueError as exc:
        raise GraspServerError(f"server response was not JSON: {exc}") from exc


def poll_capture_request(server_url: str, *, timeout_s: float = 2.0) -> bool:
    """Poll ``GET /poll_capture_request``; returns True if the Web UI requested a capture."""
    url = server_url.rstrip("/") + "/poll_capture_request"
    try:
        response = requests.get(url, timeout=timeout_s)
        if response.status_code == 200:
            return bool(response.json().get("requested", False))
    except requests.RequestException:
        pass
    return False


def push_frame(server_url: str, jpeg_bytes: bytes, *, timeout_s: float = 0.5) -> None:
    """POST a JPEG frame to ``/push_frame`` for the live Web UI stream (best-effort)."""
    url = server_url.rstrip("/") + "/push_frame"
    try:
        requests.post(url, files={"frame": ("frame.jpg", jpeg_bytes, "image/jpeg")}, timeout=timeout_s)
    except requests.RequestException:
        pass


def poll_publish(server_url: str, *, timeout_s: float = 2.0) -> "GraspResult | None":
    """Poll ``GET /poll_publish``; returns a GraspResult if the UI triggered a publish, else None."""
    url = server_url.rstrip("/") + "/poll_publish"
    try:
        response = requests.get(url, timeout=timeout_s)
    except requests.RequestException:
        return None
    if response.status_code == 404:
        return None
    if response.status_code != 200:
        return None
    try:
        payload = response.json()
    except ValueError:
        return None
    if not isinstance(payload, dict):
        return None
    return GraspResult.from_json(payload)


def get_health(server_url: str, *, timeout_s: float = 5.0) -> dict[str, Any]:
    """Probe ``GET /health``; useful at node startup to fail fast on misconfiguration."""
    url = server_url.rstrip("/") + "/health"
    response = requests.get(url, timeout=timeout_s)
    response.raise_for_status()
    payload = response.json()
    if not isinstance(payload, dict):
        raise GraspServerError(
            f"health response root must be an object, got {type(payload).__name__}",
            body=payload,
        )
    return payload
