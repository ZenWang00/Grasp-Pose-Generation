"""Pure synthetic-host-v1 mathematics; no clocks, devices or motion authority."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from ssc_core import NUMERIC_OFFLINE_V1, Pose, SessionConfig, parse_record, to_dict
from ssc_core.host_constraints import ConstraintMargin, HostConstraintResult, MeasuredStateView


JOINT_NAMES = tuple(f"q{i}" for i in range(1, 7))
ORIGIN = (.4, 0., .3)
SCALE = .2
ANGLE_LIMIT = math.pi / 3


def _pose(value):
    return parse_record(Pose, to_dict(value), policy=NUMERIC_OFFLINE_V1)


def _joints(values):
    if len(values) != 6 or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError("synthetic_joint_values")
    return np.asarray(values, dtype=np.float64)


def proxy_forward(joints) -> Pose:
    """The declared rotary proxy, not real-arm forward kinematics."""
    q = _joints(joints)
    position = np.asarray(ORIGIN) + SCALE * np.sin(q[:3])
    rotation = Rotation.from_euler("xyz", q[3:])
    return _pose(Pose(tuple(position.tolist()), tuple(rotation.as_quat().tolist())))


def proxy_inverse(pose: Pose) -> tuple[float, ...]:
    """Exact principal inverse domain; never clip an infeasible coordinate."""
    pose = _pose(pose)
    scaled = (np.asarray(pose.position_xyz) - ORIGIN) / SCALE
    if np.any(np.abs(scaled) > 1):
        raise ValueError("synthetic_inverse_domain")
    matrix = Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
    sine = -float(matrix[2, 0])
    if abs(sine) > 1:
        raise ValueError("synthetic_inverse_domain")
    angles = (math.atan2(matrix[2, 1], matrix[2, 2]), math.asin(sine),
              math.atan2(matrix[1, 0], matrix[0, 0]))
    return tuple(np.arcsin(scaled).tolist()) + angles


def measured_view(state) -> MeasuredStateView:
    return MeasuredStateView(state.frame_id, state.tcp_frame, state.tcp_pose,
                             state.joint_names, state.joint_positions_rad)


def _configuration(configuration):
    config = parse_record(SessionConfig, to_dict(configuration), policy=NUMERIC_OFFLINE_V1)
    if (config.control_frame, config.tcp_frame, config.robot_model_id,
            config.motion_constraint_set_id, config.validation_policy_id) != (
            "synthetic_base", "synthetic_tcp", "synthetic-rotary-v1",
            "synthetic-host-v1", "numeric-offline-v1"):
        raise ValueError("synthetic_configuration_mismatch")
    return config


def _margin(name, value):
    return ConstraintMargin(name, float(value), name)


@dataclass(frozen=True, slots=True)
class SyntheticHostChecker:
    """Consumer-owned implementation of the server HostConstraintChecker API.

    Invalid inputs raise; every valid mathematical report has finite named
    margins. Both solver and gate must treat exceptions as failure.
    """

    def __call__(self, *, candidate_pose, measured_state, u_xy, dt_s,
                 configuration) -> HostConstraintResult:
        config = _configuration(configuration)
        candidate = _pose(candidate_pose)
        measured = _pose(measured_state.tcp_pose)
        if (measured_state.frame_id, measured_state.tcp_frame) != (config.control_frame, config.tcp_frame):
            raise ValueError("synthetic_frame_mismatch")
        if measured_state.joint_names != JOINT_NAMES:
            raise ValueError("synthetic_joint_names")
        q = _joints(measured_state.joint_positions_rad)
        forward = proxy_forward(tuple(q.tolist()))
        position_error = np.linalg.norm(np.asarray(forward.position_xyz) - measured.position_xyz)
        rotation_error = (Rotation.from_quat(forward.quaternion_xyzw) *
                          Rotation.from_quat(measured.quaternion_xyzw).inv()).magnitude()
        if position_error > 1e-9 or rotation_error > 1e-8:
            raise ValueError("synthetic_feedback_mismatch")
        margins = []
        for prefix, values in (("measured_joint", q), ("candidate_joint", proxy_inverse(candidate))):
            for index, value in enumerate(values, 1):
                margins.extend((_margin(f"{prefix}_{index}_lower", (value + ANGLE_LIMIT) / ANGLE_LIMIT),
                                _margin(f"{prefix}_{index}_upper", (ANGLE_LIMIT - value) / ANGLE_LIMIT)))
        for axis, value, lower, upper in zip("xyz", candidate.position_xyz, (.3, -.1, .2), (.5, .1, .4)):
            margins.extend((_margin(f"workspace_{axis}_lower", (value - lower) / SCALE),
                            _margin(f"workspace_{axis}_upper", (upper - value) / SCALE)))
        start, end = np.asarray(measured.position_xyz), np.asarray(candidate.position_xyz)
        delta, center = end - start, np.asarray((.45, .07, .30))
        squared = float(delta @ delta)
        fraction = min(1., max(0., float((center - start) @ delta) / squared)) if squared else 0.
        distance = float(np.linalg.norm(start + fraction * delta - center))
        margins.append(_margin("excluded_sphere_segment", (distance - .0201) / SCALE))
        return HostConstraintResult(tuple(margins), policy=NUMERIC_OFFLINE_V1)


def basic_step_constraints(*, candidate_pose, measured_state, u_xy, dt_s,
                           configuration) -> HostConstraintResult:
    """Recompute section 4 norm, half-space and six box bounds from live data."""
    config = _configuration(configuration)
    candidate, measured = _pose(candidate_pose), _pose(measured_state.tcp_pose)
    if (type(dt_s) not in (int, float) or not math.isfinite(dt_s) or dt_s <= 0 or
            len(u_xy) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) for v in u_xy)):
        raise ValueError("invalid_step_inputs")
    magnitude = math.hypot(*u_xy)
    if magnitude > 1:
        raise ValueError("invalid_step_inputs")
    x = (np.asarray(candidate.position_xyz) - measured.position_xyz) / (config.max_linear_speed_mps * dt_s)
    r = (Rotation.from_quat(candidate.quaternion_xyzw) *
         Rotation.from_quat(measured.quaternion_xyzw).inv()).as_rotvec() / (config.max_angular_speed_radps * dt_s)
    direction = np.asarray(u_xy) / magnitude if magnitude else np.zeros(2)
    margins = [_margin("translation_norm", magnitude ** 2 - x @ x),
               _margin("rotation_norm", magnitude ** 2 - r @ r),
               _margin("input_half_space", direction @ x[:2])]
    for prefix, values in (("translation", x), ("rotation", r)):
        for axis, value in zip("xyz", values):
            margins.extend((_margin(f"{prefix}_{axis}_lower", 1 + value),
                            _margin(f"{prefix}_{axis}_upper", 1 - value)))
    return HostConstraintResult(tuple(margins), policy=NUMERIC_OFFLINE_V1)
