from dataclasses import replace
import math

import pytest
from scipy.spatial.transform import Rotation

from ssc_core import NUMERIC_CANDIDATES_V2, Pose, SessionConfig, parse_record
from ssc_core.host_constraints import MeasuredStateView
from ssc_runtime import SyntheticHostChecker, basic_step_constraints, proxy_forward, proxy_inverse


@pytest.fixture
def config(catalog):
    data = next(m["payload"] for m in catalog["messages"] if m["message_type"] == "SessionConfig")
    return parse_record(SessionConfig, data, policy=NUMERIC_CANDIDATES_V2)


def pose(position=(.4, 0, .3), angles=(0, 0, 0)):
    return Pose(tuple(position), tuple(Rotation.from_euler("xyz", angles).as_quat().tolist()))


def view(p=None):
    p = p or pose()
    return MeasuredStateView("synthetic_base", "synthetic_tcp", p,
                             tuple(f"q{i}" for i in range(1, 7)), proxy_inverse(p))


def report(config, candidate=None, measured=None):
    return SyntheticHostChecker()(candidate_pose=candidate or pose(), measured_state=measured or view(),
                                  u_xy=(1., 0.), dt_s=.2, configuration=config)


def test_analytical_forward_and_inverse():
    assert proxy_forward((0.,) * 6) == pose()
    for q in ((.1, -.2, .3, -.4, .5, -.6), (math.pi / 3,) * 6, (-math.pi / 3,) * 6):
        assert proxy_inverse(proxy_forward(q)) == pytest.approx(q, abs=1e-14)
    target = proxy_forward((math.pi / 6, 0., 0., 0., 0., math.pi / 6))
    assert target.position_xyz == pytest.approx((.5, 0, .3))
    assert Rotation.from_quat(target.quaternion_xyzw).apply([1, 0, 0]) == pytest.approx([math.sqrt(3)/2, .5, 0])


def test_zero_length_segment_and_named_finite_margins(config):
    result = report(config)
    assert result.feasible and result.violation_reasons == ()
    values = {m.name: m.value for m in result.margins}
    assert len(values) == 31
    assert values["workspace_x_upper"] == pytest.approx(.5)
    assert values["excluded_sphere_segment"] == pytest.approx((math.hypot(.05, .07) - .0201) / .2)
    assert all(math.isfinite(m.value) for m in result.margins)


def test_entire_segment_not_only_endpoints(config):
    start, end = pose((.42, .07, .3)), pose((.48, .07, .3))
    assert report(config, start, view(start)).feasible
    assert report(config, end, view(end)).feasible
    crossing = report(config, end, view(start))
    assert crossing.violation_reasons == ("excluded_sphere_segment",)
    assert next(m.value for m in crossing.margins if m.name == "excluded_sphere_segment") == pytest.approx(-.1005)


@pytest.mark.parametrize("axis,low,high", [(0,.3,.5), (1,-.1,.1), (2,.2,.4)])
def test_workspace_faces_and_tolerance(config, axis, low, high):
    for value in (low, high):
        point = [.4, 0., .3]
        point[axis] = value
        assert report(config, pose(point)).feasible
    for value in (low - .001, high + .001):
        point = [.4, 0., .3]
        point[axis] = value
        assert not report(config, pose(point)).feasible


@pytest.mark.parametrize("bad", [(.60000000001,0,.3), (.4,0,.50000000001)])
def test_inverse_domain_is_not_clipped(config, bad):
    with pytest.raises(ValueError, match="inverse_domain"):
        report(config, pose(bad))


def test_joint_bounds_and_measured_fk_consistency(config):
    for axis in range(3):
        angles = [0., 0., 0.]
        angles[axis] = math.pi/3 + .01
        assert not report(config, pose(angles=angles)).feasible
    with pytest.raises(ValueError, match="feedback_mismatch"):
        report(config, measured=replace(view(), joint_positions_rad=(.1,0,0,0,0,0)))
    with pytest.raises(ValueError, match="feedback_mismatch"):
        report(config, measured=replace(view(), tcp_pose=pose(angles=(0,0,.001))))


@pytest.mark.parametrize("change,reason", [("names", "joint_names"), ("frame", "frame_mismatch"),
    ("count", "joint_values"), ("finite", "joint_values"), ("quat", "unit_norm")])
def test_bad_state_is_rejected(config, change, reason):
    measured = view()
    if change == "names": measured = replace(measured, joint_names=tuple(reversed(measured.joint_names)))
    if change == "frame": measured = replace(measured, frame_id="camera")
    if change == "count": measured = replace(measured, joint_positions_rad=(0.,)*5)
    if change == "finite": measured = replace(measured, joint_positions_rad=(float("nan"),)*6)
    if change == "quat": measured = replace(measured, tcp_pose=Pose((.4,0,.3), (0,0,0,2)))
    with pytest.raises(ValueError, match=reason): report(config, measured=measured)


@pytest.mark.parametrize("field", ["control_frame", "tcp_frame", "robot_model_id", "motion_constraint_set_id", "validation_policy_id"])
def test_configuration_identity_is_host_owned(config, field):
    with pytest.raises(ValueError, match="configuration_mismatch"):
        report(replace(config, **{field: "foreign"}))


@pytest.mark.parametrize("position,angle,u,expected", [
    ((.41,0,.3), 0, (1,0), True), ((.4101,0,.3), 0, (1,0), False),
    ((.405,0,.3), 0, (.5,0), True), ((.4051,0,.3), 0, (.5,0), False),
    ((.405,0,.3), 0, (-1,0), False), ((.4,0,.3), .05, (.5,0), True),
    ((.4,0,.3), .0501, (.5,0), False), ((.4,0,.3), 0, (0,0), True),
    ((.4001,0,.3), 0, (0,0), False),
])
def test_input_scaled_translation_rotation_and_direction(config, position, angle, u, expected):
    result = basic_step_constraints(candidate_pose=pose(position, (0,0,angle)), measured_state=view(),
                                    u_xy=u, dt_s=.2, configuration=config)
    assert result.feasible is expected
    assert len(result.margins) == 15


def test_rotation_increment_is_control_frame_and_current_measured(config):
    measured = pose(angles=(.3, -.2, .1))
    target_rotation = Rotation.from_rotvec([.01, .02, -.03]) * Rotation.from_quat(measured.quaternion_xyzw)
    target = Pose(measured.position_xyz, tuple(target_rotation.as_quat().tolist()))
    result = basic_step_constraints(candidate_pose=target, measured_state=view(measured),
                                    u_xy=(1,0), dt_s=.2, configuration=config)
    margins = {m.name: m.value for m in result.margins}
    assert margins["rotation_norm"] == pytest.approx(1 - .14)
    assert margins["rotation_z_upper"] == pytest.approx(1.3)
