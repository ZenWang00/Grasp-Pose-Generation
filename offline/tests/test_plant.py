"""Synthetic state/provenance tests; no hardware or real solver claims."""

from dataclasses import asdict, replace
import json
import math

import numpy as np
import pytest
from pathlib import Path
import runpy

# Pytest 7 importlib mode may create a synthetic `tests` package that has no
# sibling search path. Load this pure test helper without altering sys.path.
selection_and_evaluation = runpy.run_path(str(Path(__file__).with_name("helpers.py")))["selection_and_evaluation"]
from scipy.spatial.transform import Rotation

from ssc_core import NUMERIC_CANDIDATES_V2, Pose, SessionConfig, parse_message, parse_record, to_dict
from ssc_runtime import (AcceptedTarget, CommandGate, ControllerError, LifecycleCoordinator,
                         LifecycleState, PLANT_TICK_NS, SessionRuntime, SnapshotCoordinator,
                         SyntheticController, proxy_forward, proxy_inverse)

NS = 1_000_000_000
T0 = 10 * NS
CLOCK = "synthetic-monotonic-epoch-1"
SESSION = "session-offline-v1"


class Clock:
    def __init__(self): self.now = T0
    def __call__(self): return self.now


class Canceller:
    def cancel(self, control_epoch, reason): pass


def stamp(ns):
    return {"clock_id": CLOCK, "sec": ns // NS, "nanosec": ns % NS}


def envelope(kind, sequence, ns, payload):
    return {"message_type": kind, "meta": {
        "schema_version": "2.0", "session_id": SESSION, "message_id": f"{kind}-{sequence}",
        "source_id": kind, "sequence": sequence, "produced_at": stamp(ns)}, "payload": payload}


@pytest.fixture
def config(catalog):
    return parse_record(SessionConfig, next(m["payload"] for m in catalog["messages"]
                        if m["message_type"] == "SessionConfig"), policy=NUMERIC_CANDIDATES_V2)


@pytest.fixture
def setup(config):
    clock = Clock()
    plant = SyntheticController(config, session_id=SESSION, clock_ns=clock, clock_id=CLOCK)
    return plant, clock


def target(plant, clock, *, seq=1, u=(1., 0.), position=(.409375, 0., .3),
           quaternion=(0., 0., 0., 1.), state=None):
    message = parse_message(envelope("ControlResult", seq, clock.now, {
        "result_id": f"result-{seq}", "snapshot_id": f"snapshot-{seq}", "selection_id": f"selection-{seq}",
        "control_epoch": 1, "command_seq": seq, "source_joystick_seq": 1, "source_robot_state_seq": 1,
        "status": "move", "evaluation_valid": True, "solver_status": "solved",
        "constraints_satisfied": False, "solve_time_s": .01,
        "candidate_evaluation": {"candidate_id": "distance", "distance_m": .000625, "raw_g": .000625,
                                    "normalization_scale": .01, "penalty": 1/256},
        "next_tcp_pose": {"position_xyz": list(position), "quaternion_xyzw": [float(v) for v in quaternion]},
        "horizon_s": .2}), policy=NUMERIC_CANDIDATES_V2)
    return AcceptedTarget(message, clock.now, clock.now + 200_000_000, u,
                          plant.sample() if state is None else state)


def tick(plant, clock):
    clock.now += PLANT_TICK_NS
    plant.advance()
    return plant.sample()


def test_receipt_does_not_manufacture_feedback_and_first_tick_matches_profile(setup):
    plant, clock = setup
    initial = plant.sample()
    command = target(plant, clock)
    plant.apply(command)
    assert plant.sample() is initial
    assert plant.target == command
    observed = tick(plant, clock)
    assert observed.payload.tcp_pose.position_xyz == pytest.approx((.401, 0., .3), abs=1e-12)
    assert observed.payload.joint_positions_rad[0] == pytest.approx(math.asin(.001/.2), abs=1e-12)
    assert observed.payload.joint_velocities_radps[0] == pytest.approx(math.asin(.001/.2)/.02)
    assert observed.payload.tcp_pose != command.result.payload.next_tcp_pose
    assert observed.payload.state_source == "fk_from_measured_joints"
    assert observed.payload.joint_names == tuple(f"q{i}" for i in range(1, 7))
    assert to_dict(observed.payload.sampled_at) == stamp(T0 + PLANT_TICK_NS)
    assert observed.meta.produced_at == observed.payload.sampled_at
    assert observed.meta.sequence == initial.meta.sequence + 1
    assert parse_message(json.loads(json.dumps(to_dict(observed))), policy=NUMERIC_CANDIDATES_V2) == observed


def test_lag_and_translation_rotation_share_speed_limited_gamma(setup):
    plant, clock = setup
    command = target(plant, clock, u=(.5, 0.), position=(.404, 0., .3),
                     quaternion=tuple(Rotation.from_rotvec((0., 0., .05)).as_quat()))
    plant.apply(command)
    # Angular limit sets gamma=.1, so translation is .0004, below its .0005 bound.
    state = tick(plant, clock).payload
    assert state.tcp_pose.position_xyz[0] == pytest.approx(.4004, abs=1e-12)
    assert state.joint_positions_rad[5] == pytest.approx(.005, abs=1e-12)
    for _ in range(7):
        old = state
        state = tick(plant, clock).payload
        linear = np.linalg.norm(np.subtract(state.tcp_pose.position_xyz, old.tcp_pose.position_xyz))
        angle = (Rotation.from_quat(state.tcp_pose.quaternion_xyzw) *
                 Rotation.from_quat(old.tcp_pose.quaternion_xyzw).inv()).magnitude()
        assert linear <= .0005 + 1e-12
        assert angle <= .005 + 1e-12
    # On the final live tick, the .5 lag cap halves the remaining error.
    old = state
    state = tick(plant, clock).payload
    assert .404 - state.tcp_pose.position_xyz[0] == pytest.approx((.404-old.tcp_pose.position_xyz[0])*.5)


@pytest.mark.parametrize("distance", [0., .0008])
def test_half_lag_and_zero_distance_denominators(setup, distance):
    plant, clock = setup
    plant.apply(target(plant, clock, position=(.4+distance, 0., .3)))
    for step in range(1, 5):
        observed = tick(plant, clock)
        assert observed.payload.tcp_pose.position_xyz[0] == pytest.approx(
            .4 + distance * (1 - .5 ** step), abs=1e-12)


def test_orientation_is_left_multiplied_and_feedback_matches_proxy(config):
    clock = Clock()
    plant = SyntheticController(config, session_id=SESSION, clock_ns=clock, clock_id=CLOCK,
                                initial_joints=(0., 0., 0., .2, -.1, .3))
    initial = plant.sample().payload
    start = Rotation.from_quat(initial.tcp_pose.quaternion_xyzw)
    delta = np.array((.04, .03, .02))
    desired = Rotation.from_rotvec(delta) * start
    plant.apply(target(plant, clock, position=(.4, 0., .3), quaternion=tuple(desired.as_quat())))
    state = tick(plant, clock).payload
    expected = Rotation.from_rotvec(delta * (.01 / np.linalg.norm(delta))) * start
    assert (Rotation.from_quat(state.tcp_pose.quaternion_xyzw) * expected.inv()).magnitude() < 1e-8
    assert proxy_forward(state.joint_positions_rad) == state.tcp_pose


def test_replacement_reversal_and_duplicate_never_renew_expiry(setup):
    plant, clock = setup
    first = target(plant, clock)
    plant.apply(first)
    observed = tick(plant, clock)
    second = target(plant, clock, seq=2, u=(-1., 0.), position=(.396, 0., .3))
    plant.apply(second)
    assert plant.sample() is observed
    for command in (first, second, replace(second, expires_ns=second.expires_ns+PLANT_TICK_NS)):
        with pytest.raises(ControllerError): plant.apply(command)
        assert plant.target == second
    assert tick(plant, clock).payload.tcp_pose.position_xyz[0] == pytest.approx(.4, abs=1e-12)
    assert [e.detail for e in plant.events if e.operation == "apply" and e.outcome == "ok"] == [None, "replaces:result-1"]


@pytest.mark.parametrize("action", ["cancel", "hold", "stop"])
def test_cancel_hold_stop_prevent_later_advancement_without_copying_pose(setup, action):
    plant, clock = setup
    old_sample = plant.sample()
    plant.apply(target(plant, clock))
    moved = tick(plant, clock)
    if action == "hold": plant.hold(old_sample)  # fresh, but behind current state
    else: getattr(plant, action)()
    assert plant.target is None
    for _ in range(3):
        observed = tick(plant, clock)
        assert observed.payload.tcp_pose == moved.payload.tcp_pose
        assert observed.payload.joint_velocities_radps == (0.,) * 6
    assert [e.operation for e in plant.events if e.operation in ("cancel", "hold", "stop")] == [action]


def test_expiry_equality_removes_target_and_does_not_move(setup):
    plant, clock = setup
    command = target(plant, clock)
    plant.apply(command)
    for _ in range(9): last = tick(plant, clock)
    final = tick(plant, clock)
    assert clock.now == command.expires_ns
    assert final.payload.tcp_pose == last.payload.tcp_pose
    assert final.payload.joint_velocities_radps == (0.,) * 6
    assert plant.target is None
    assert [e.at_ns for e in plant.events if e.operation == "expire"] == [command.expires_ns]


@pytest.mark.parametrize("mode", ["missing", "stale"])
def test_feedback_failures_preserve_old_sample_and_do_not_freeze_plant(setup, mode):
    plant, clock = setup
    old = plant.sample()
    plant.apply(target(plant, clock))
    plant.set_feedback_mode(mode)
    for _ in range(26):
        observed = tick(plant, clock)
        assert observed is (None if mode == "missing" else old)
    assert to_dict(old.payload.sampled_at) == stamp(T0)
    with pytest.raises(ControllerError, match="feedback_not_fresh"): plant.hold(old)
    plant.stop()  # needs no feedback
    plant.set_feedback_mode("normal")
    recovered = plant.sample()
    assert recovered.meta.sequence == old.meta.sequence + 1
    assert recovered.payload.sampled_at.sec * NS + recovered.payload.sampled_at.nanosec == clock.now
    assert recovered.payload.tcp_pose != old.payload.tcp_pose


def test_stale_without_prior_sample_does_not_invent_measurement(setup):
    plant, _ = setup
    plant.set_feedback_mode("stale")
    assert plant.sample() is None


@pytest.mark.parametrize("operation", ["apply", "cancel", "hold", "stop", "advance"])
def test_injected_submission_failures_are_latched_observable_and_do_not_commit_q(setup, operation):
    plant, clock = setup
    command = target(plant, clock)
    if operation != "apply": plant.apply(command)
    before = plant.sample()
    plant.fail_next(operation)
    with pytest.raises(ControllerError, match="injected_"):
        if operation == "apply": plant.apply(command)
        elif operation == "hold": plant.hold(before)
        elif operation == "advance": tick(plant, clock)
        else: getattr(plant, operation)()
    assert not plant.status.ready and plant.status.fault == f"injected_{operation}_failure"
    assert tick(plant, clock).payload.tcp_pose == before.payload.tcp_pose
    plant.stop()
    assert not plant.status.ready  # a successful stop does not clear the fault
    plant.clear_fault()
    assert plant.status.ready and plant.target is None
    assert any(e.operation == operation and e.outcome == "failed" for e in plant.events)


def test_internal_constraint_failure_keeps_previous_joint_state(setup):
    plant, clock = setup
    # Source feedback can be valid but lag the independent plant. A target that
    # now opposes its accepted direction must fail the actual tick's checks.
    source = plant.sample()
    plant.apply(target(plant, clock))
    moved = tick(plant, clock)
    plant.apply(target(plant, clock, seq=2, position=(.4001, 0., .3), state=source))
    with pytest.raises(ControllerError, match="input_half_space"): tick(plant, clock)
    assert plant.sample().payload.tcp_pose == moved.payload.tcp_pose
    assert plant.events[-2].joints_before == plant.events[-2].joints_after
    assert not plant.status.ready


@pytest.mark.parametrize("change", ["time", "expiry", "neutral", "nonfinite", "session", "model", "bounds", "opposition"])
def test_invalid_targets_never_install_or_modify_feedback(setup, change):
    plant, clock = setup
    command = target(plant, clock)
    old = plant.sample()
    if change == "time": command = replace(command, accepted_ns=T0-1)
    elif change == "expiry": command = replace(command, expires_ns=command.expires_ns+1)
    elif change == "neutral": command = replace(command, u_xy=(0., 0.))
    elif change == "nonfinite": command = replace(command, u_xy=(float("nan"), 0.))
    elif change == "session":
        command = replace(command, result=replace(command.result, meta=replace(command.result.meta, session_id="other")))
    elif change == "model":
        command = replace(command, measured_state=replace(old, payload=replace(old.payload, joint_positions_rad=(.1,)*6)))
    elif change == "bounds": command = target(plant, clock, position=(.6, 0., .3))
    elif change == "opposition": command = replace(command, u_xy=(-1., 0.))
    with pytest.raises(ControllerError): plant.apply(command)
    assert plant.target is None and plant.sample() is old


def test_clock_requires_one_tick_and_never_catches_up(setup):
    plant, clock = setup
    before = plant.sample()
    with pytest.raises(ControllerError, match="expected_one"): plant.advance()
    clock.now += 2 * PLANT_TICK_NS
    with pytest.raises(ControllerError, match="expected_one"): plant.advance()
    assert plant.sample() is before
    clock.now = T0
    with pytest.raises(ControllerError, match="regressing_clock"): plant.sample()


@pytest.mark.parametrize("joints", [(True, 0, 0, 0, 0, 0), (0,)*5, (0, 0, 0, 0, 0, 2),
                                   (math.asin(.05/.2), math.asin(.07/.2), 0, 0, 0, 0)])
def test_invalid_initial_model_is_rejected(config, joints):
    with pytest.raises((ValueError, TypeError)):
        SyntheticController(config, session_id=SESSION, clock_ns=Clock(), clock_id=CLOCK, initial_joints=joints)


def run_trace(config):
    clock = Clock()
    plant = SyntheticController(config, session_id=SESSION, clock_ns=clock, clock_id=CLOCK)
    plant.apply(target(plant, clock))
    first = tick(plant, clock)
    plant.apply(target(plant, clock, seq=2, u=(-1., 0.), position=(.396, 0., .3)))
    tick(plant, clock)
    plant.cancel()
    plant.hold(plant.sample())
    tick(plant, clock)
    plant.set_feedback_mode("stale")
    tick(plant, clock)
    plant.fail_next("stop")
    with pytest.raises(ControllerError): plant.stop()
    plant.stop()
    plant.clear_fault()
    plant.set_feedback_mode("normal")
    final = tick(plant, clock)
    return {"provenance": plant.provenance, "configuration": to_dict(config),
            "expected": {"first_x_m": .401, "final_x_m": .4, "tick_ns": PLANT_TICK_NS},
            "actual": {"first_x_m": first.payload.tcp_pose.position_xyz[0],
                       "final_x_m": final.payload.tcp_pose.position_xyz[0]},
            "events": [asdict(event) for event in plant.events]}


def test_deterministic_retained_plant_trace(config, tmp_path):
    first, second = run_trace(config), run_trace(config)
    assert first == second
    assert first["actual"]["first_x_m"] == pytest.approx(first["expected"]["first_x_m"], abs=1e-9)
    assert first["actual"]["final_x_m"] == pytest.approx(first["expected"]["final_x_m"], abs=1e-9)
    (tmp_path / "plant-trace.json").write_text(json.dumps(first, indent=2, allow_nan=False) + "\n")


@pytest.fixture
def integrated(build):
    clock = Clock()
    store, request, records, _ = build()
    config = parse_record(SessionConfig, records["SessionConfig"], policy=NUMERIC_CANDIDATES_V2)
    plant = SyntheticController(config, session_id=SESSION, clock_ns=clock, clock_id=CLOCK)
    coord = SnapshotCoordinator(store, clock_ns=clock, clock_id=CLOCK,
        joystick_source_id="JoystickSample", robot_state_source_id=plant.source_id, mapping_id="normalized-xy-v1")
    gate = CommandGate(coord, plant)
    lifecycle = LifecycleCoordinator(lambda _: SessionRuntime(store, coord, gate), session_id=SESSION,
                                     worker_canceller=Canceller())
    sequence = 0

    def joystick(u):
        nonlocal sequence
        sequence += 1
        return lifecycle.on_input(envelope("JoystickSample", sequence, clock.now,
            {"sampled_at": stamp(clock.now), "mapping_id": "normalized-xy-v1", "u_xy": list(u), "enabled": True}))

    lifecycle.on_input(plant.sample())
    joystick((0., 0.))
    assert lifecycle.initialize(request).status == "activated"
    lifecycle.controller_status(**asdict(plant.status))
    lifecycle.computation_status(ready=True, reason="test worker ready")
    joystick((1., 0.))
    assert lifecycle.state == LifecycleState.ACTIVE
    dispatch = lifecycle.dispatch_cycle()
    command = target(plant, clock)
    result = to_dict(command.result)
    result["payload"].update(snapshot_id=dispatch.inputs.snapshot.snapshot_id,
        control_epoch=dispatch.inputs.snapshot.control_epoch, command_seq=dispatch.command_seq,
        source_joystick_seq=dispatch.inputs.joystick.message.meta.sequence,
        source_robot_state_seq=dispatch.inputs.robot_state.message.meta.sequence,
        candidate_evaluation=None)
    selection, evaluation = selection_and_evaluation(dispatch.inputs, "selection-1", result["payload"]["next_tcp_pose"]["position_xyz"])
    result["payload"]["candidate_evaluation"] = evaluation
    selection = envelope("CandidateSelection", 1, clock.now, selection)
    return plant, clock, lifecycle, joystick, result, selection


def test_real_gate_apply_then_local_release_and_late_result(integrated):
    plant, clock, lc, joystick, result, selection = integrated
    decision = lc.submit_result(result, selection=selection)
    assert decision.disposition == "accepted_move"
    assert plant.sample().payload.tcp_pose.position_xyz[0] == .4
    observed = tick(plant, clock)
    lc.on_input(observed)
    joystick((0., 0.))
    assert lc.state == LifecycleState.PAUSED and plant.target is None
    assert [e.operation for e in plant.events if e.operation in ("cancel", "hold", "stop")] == ["cancel", "hold"]
    before_events = plant.events
    assert lc.submit_result(result, selection=selection).disposition == "discarded"
    assert plant.events == before_events
    assert tick(plant, clock).payload.tcp_pose == observed.payload.tcp_pose


@pytest.mark.parametrize("failure", ["apply", "stop", "controller"])
def test_controller_failures_reach_lifecycle_fault_and_require_rearm(integrated, failure):
    plant, clock, lc, joystick, result, selection = integrated
    if failure == "apply":
        plant.fail_next("apply")
        assert lc.submit_result(result, selection=selection).lifecycle_request == "FAULT"
    else:
        assert lc.submit_result(result, selection=selection).disposition == "accepted_move"
        tick(plant, clock)
        if failure == "stop": plant.fail_next("stop")
        plant.inject_controller_fault()
        lc.controller_status(**asdict(plant.status))
    assert lc.state == LifecycleState.FAULT
    plant.stop()
    plant.clear_fault()
    lc.controller_status(**asdict(plant.status))
    assert lc.state == LifecycleState.FAULT
    lc.rearm("explicit simulated operator rearm")
    assert lc.state == LifecycleState.UNINITIALIZED and plant.target is None
