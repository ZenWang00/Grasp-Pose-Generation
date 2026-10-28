from concurrent.futures import ThreadPoolExecutor
import json
from threading import Event

import pytest
from pathlib import Path
import runpy

# Pytest 7 importlib mode may create a synthetic `tests` package that has no
# sibling search path. Load this pure test helper without altering sys.path.
selection_and_evaluation = runpy.run_path(str(Path(__file__).with_name("helpers.py")))["selection_and_evaluation"]
from ssc_core import NUMERIC_CANDIDATES_V2, Pose, parse_record, to_dict
from ssc_core.host_constraints import MeasuredStateView
from ssc_runtime import (CommandGate, SnapshotCoordinator, SyntheticHostChecker,
                         basic_step_constraints, proxy_forward, proxy_inverse)

NS = 1_000_000_000
CLOCK = "synthetic-monotonic-epoch-1"


class Clock:
    now = 10 * NS

    def __call__(self): return self.now


def stamp(ns):
    return {"clock_id": CLOCK, "sec": ns // NS, "nanosec": ns % NS}


def envelope(kind, sequence, ns, payload):
    return {"message_type": kind, "meta": {
        "schema_version": "2.0", "session_id": "session-offline-v1",
        "message_id": f"{kind}-{sequence}", "source_id": kind,
        "sequence": sequence, "produced_at": stamp(ns)}, "payload": payload}


def joystick(seq=1, ns=10 * NS, **changes):
    payload = {"sampled_at": stamp(ns), "mapping_id": "normalized-xy-v1", "u_xy": [1,0], "enabled": True}
    payload.update(changes)
    return envelope("JoystickSample", seq, ns, payload)


def state(seq, ns, **values):
    return envelope("RobotState", seq, ns, {"sampled_at": stamp(ns), "frame_id": "synthetic_base",
                                           "tcp_frame": "synthetic_tcp", **values})


def activate(store, request):
    return store.publish(store.prepare(store.begin(request))).bundle


def move(dispatch, *, result_id):
    source = dispatch.inputs
    return envelope("ControlResult", dispatch.command_seq, 10 * NS, {
        "result_id": result_id, "snapshot_id": source.snapshot.snapshot_id, "selection_id": "pending",
        "control_epoch": source.snapshot.control_epoch, "command_seq": dispatch.command_seq,
        "source_joystick_seq": source.joystick.message.meta.sequence,
        "source_robot_state_seq": source.robot_state.message.meta.sequence,
        "status": "move", "evaluation_valid": True, "solver_status": "solved",
        "constraints_satisfied": False, "solve_time_s": .01, "candidate_evaluation": None,
        "next_tcp_pose": {"position_xyz": [.405,0,.3], "quaternion_xyzw": [0,0,0,1]},
        "horizon_s": source.snapshot.dt_s})


class SpyAdapter:
    def __init__(self):
        self.calls = []
        self.targets = []
        self.fail = set()

    def _call(self, name):
        self.calls.append(name)
        if name in self.fail:
            raise RuntimeError(name + " failed")

    def apply(self, target):
        self._call("apply")
        self.targets.append(target)

    def cancel(self): self._call("cancel")
    def hold(self, measured_state):
        self._call("hold")
        self.held = measured_state
    def stop(self): self._call("stop")


def feedback(seq=1, ns=10 * NS, *, position=(.4, 0, .3), rotation=(0, 0, 0, 1), **changes):
    pose = Pose(tuple(position), tuple(rotation))
    return state(seq, ns, tcp_pose=to_dict(pose), joint_names=[f"q{i}" for i in range(1, 7)],
                 joint_positions_rad=list(proxy_inverse(pose)), state_source="fk_from_measured_joints", **changes)


@pytest.fixture
def gate_setup(build):
    store, request, records, _ = build()
    bundle = activate(store, request)
    clock = Clock()
    coordinator = SnapshotCoordinator(store, clock_ns=clock, clock_id=CLOCK,
        joystick_source_id="JoystickSample", robot_state_source_id="RobotState", mapping_id="normalized-xy-v1", control_epoch=3)
    coordinator.receive(joystick())
    coordinator.receive(feedback())
    adapter = SpyAdapter()
    gate = CommandGate(coordinator, adapter)
    gate.set_authority(control_epoch=3, motion_enabled=True, controller_ready=True)
    return gate, coordinator, clock, adapter, store, bundle


def proposal(coordinator, *, frozen=None, position=(.405, 0, .3), hold=False):
    dispatch = coordinator.dispatch(frozen or coordinator.assemble())
    result_id = "proposal-" + str(dispatch.command_seq)
    result = move(dispatch, result_id=result_id)
    selection_id = "selection-" + str(dispatch.command_seq)
    result["payload"]["selection_id"] = selection_id
    result["payload"]["next_tcp_pose"]["position_xyz"] = list(position)
    result["payload"]["next_tcp_pose"]["quaternion_xyzw"] = list(dispatch.inputs.robot_state.message.payload.tcp_pose.quaternion_xyzw)
    selection, evaluation = selection_and_evaluation(dispatch.inputs, selection_id, position)
    result["payload"]["candidate_evaluation"] = evaluation
    result["payload"]["constraints_satisfied"] = evaluation["raw_g"] <= 1e-9
    if hold:
        result["payload"].update(status="hold", reason="worker computation failure",
                                 evaluation_valid=False, solver_status="error", constraints_satisfied=None)
        result["payload"].pop("next_tcp_pose")
        result["payload"].pop("horizon_s")
    selection = envelope("CandidateSelection", dispatch.command_seq, 10 * NS, selection)
    return result, selection, dispatch


def submit(gate, triple):
    return gate.submit(triple[0], selection=triple[1])


def test_valid_positive_residual_newer_input_and_applied_history(gate_setup):
    gate, coord, clock, spy, _, _ = gate_setup
    candidate = proposal(coord, position=(.409375, 0, .3))
    clock.now += 40_000_000
    coord.receive(joystick(2, clock.now))
    coord.receive(feedback(2, clock.now, position=(.401, 0, .3)))
    decision = submit(gate, candidate)
    assert decision.proposal.command_accepted and decision.disposition == "accepted_move"
    assert spy.calls == ["apply"] and decision.observed_execution is None
    assert spy.targets[0].expires_ns == clock.now + 200_000_000
    assert spy.targets[0].measured_state.meta.sequence == 2
    assert decision.applied.result.payload.source_joystick_seq == 1
    assert not decision.applied.result.payload.constraints_satisfied
    assert coord.assemble().previous_applied is decision.applied
    assert gate.last_eligible_result_ns == clock.now


@pytest.mark.parametrize("hold", [False, True])
@pytest.mark.parametrize("case,reason", [("expired", "snapshot_expired"), ("epoch", "old_epoch"),
    ("sequence", "forged_command_sequence"), ("session", "invalid_result"),
    ("context", "superseded_bundle"), ("unknown", "unknown_dispatch")])
def test_obsolete_results_never_act_or_renew_liveness(gate_setup, hold, case, reason):
    gate, coord, clock, spy, store, _ = gate_setup
    candidate = proposal(coord, hold=hold)
    if case == "expired": clock.now = candidate[2].expires_ns
    if case == "epoch": coord.set_control_epoch(4)
    if case == "sequence": candidate[0]["payload"]["command_seq"] += 100
    if case == "session": candidate[0]["meta"]["session_id"] = "old-session"
    if case == "context": store.invalidate("new context")
    if case == "unknown": candidate[0]["payload"]["snapshot_id"] = "unknown"
    decision = submit(gate, candidate)
    assert decision.disposition == "discarded" and reason in decision.proposal.reason
    assert spy.calls == [] and gate.last_eligible_result_ns is None
    assert coord._previous is None


@pytest.mark.parametrize("hold", [False, True])
def test_old_and_duplicate_proposals_preserve_newer_target(gate_setup, hold):
    gate, coord, clock, spy, *_ = gate_setup
    old = proposal(coord, hold=hold)
    new = proposal(coord)
    accepted = submit(gate, new)
    clock.now += 1
    for candidate in (old, new):
        assert submit(gate, candidate).disposition == "discarded"
    assert spy.calls == ["apply"]
    assert coord._previous is accepted.applied
    assert gate.last_eligible_result_ns == 10 * NS


@pytest.mark.parametrize("hold", [False, True])
def test_later_dispatch_number_does_not_rescue_older_snapshot(gate_setup, hold):
    gate, coord, _, spy, *_ = gate_setup
    older = coord.assemble()
    newer = proposal(coord)
    old_late_dispatch = proposal(coord, frozen=older, hold=hold)
    assert old_late_dispatch[2].command_seq > newer[2].command_seq
    assert submit(gate, newer).proposal.command_accepted
    decision = submit(gate, old_late_dispatch)
    assert decision.proposal.reason == "superseded_snapshot" and spy.calls == ["apply"]


@pytest.mark.parametrize("change,reason", [
    ("horizon", "horizon_mismatch"), ("echo", "source_sequence_mismatch"),
    ("reversal", "input_half_space"), ("smaller", "translation_norm"),
    ("release", "motion_input_disabled"), ("disabled", "motion_input_disabled"),
    ("selection_snapshot", "missing_reference"), ("estimator", "selection_snapshot_mismatch"),
    ("candidate", "evaluation_candidate_mismatch"), ("scale", "evaluation_scale_mismatch"),
    ("penalty", "evaluation_metric_mismatch"), ("missing_candidate", "evaluation_candidate_mismatch"),
    ("workspace", "workspace_x_upper"), ("feedback_model", "synthetic_feedback_mismatch"),
])
def test_current_failures_reject_move_and_submit_separate_local_hold(gate_setup, change, reason):
    gate, coord, clock, spy, *_ = gate_setup
    if change == "workspace":
        coord.receive(feedback(2, clock.now, position=(.499, 0, .3)))
    candidate = proposal(coord, position=(.504, 0, .3) if change == "workspace" else (.405, 0, .3))
    result, selection, dispatch = candidate
    if change == "horizon": result["payload"]["horizon_s"] = .1
    if change == "echo": result["payload"]["source_joystick_seq"] += 1
    if change in ("reversal", "smaller", "release", "disabled"):
        u = {"reversal": [-1, 0], "smaller": [.1, 0], "release": [0, 0], "disabled": [1, 0]}[change]
        coord.receive(joystick(2, clock.now, u_xy=u, enabled=change != "disabled"))
    if change == "selection_snapshot": selection["payload"]["snapshot_id"] = "unknown"
    if change == "estimator": selection["payload"]["estimator_config_id"] = "unapproved"
    if change == "candidate": result["payload"]["candidate_evaluation"]["candidate_id"] = "foreign"
    if change == "scale": result["payload"]["candidate_evaluation"]["normalization_scale"] = 1
    if change == "penalty": result["payload"]["candidate_evaluation"]["penalty"] = 99
    if change == "missing_candidate": result["payload"]["candidate_evaluation"] = None
    if change == "feedback_model":
        bad = feedback(2, clock.now)
        bad["payload"]["joint_positions_rad"][0] = .5
        coord.receive(bad)
    decision = submit(gate, candidate)
    assert not decision.proposal.command_accepted and reason in decision.proposal.reason
    assert spy.calls == ["cancel", "stop" if change == "feedback_model" else "hold"]
    assert decision.applied.result.payload.status == "hold"
    assert decision.applied.result.payload.snapshot_id is None
    assert decision.applied.result.payload.command_seq > dispatch.command_seq
    assert coord.assemble().previous_applied is decision.applied
    assert gate.last_eligible_result_ns is None and decision.lifecycle_request == "PAUSED"


def test_current_hold_has_causal_order_and_duplicate_cannot_renew(gate_setup):
    gate, coord, clock, spy, *_ = gate_setup
    candidate = proposal(coord, hold=True)
    decision = submit(gate, candidate)
    assert decision.proposal.command_accepted and decision.disposition == "accepted_hold"
    assert decision.applied.source is candidate[2].inputs
    assert spy.calls == ["cancel", "hold"]
    clock.now += 1
    assert submit(gate, candidate).disposition == "discarded"
    assert gate.last_eligible_result_ns == 10 * NS


@pytest.mark.parametrize("stale_kind,actions", [("JoystickSample", ["cancel", "hold"]),
                                              ("RobotState", ["cancel", "stop"])])
def test_fresh_receipt_of_old_source_is_not_fresh(gate_setup, stale_kind, actions):
    gate, coord, clock, spy, *_ = gate_setup
    # The snapshot is new, but acquisition is near its independent age limit.
    clock.now += 490_000_000
    candidate = proposal(coord)
    clock.now += 20_000_000
    fresh = feedback if stale_kind == "JoystickSample" else joystick
    old = joystick if stale_kind == "JoystickSample" else feedback
    coord.receive(fresh(2, clock.now))
    record = old(2, 10 * NS)
    record["meta"]["produced_at"] = stamp(clock.now)
    coord.receive(record)
    decision = submit(gate, candidate)
    assert not decision.proposal.command_accepted and "stale_input" in decision.proposal.reason
    assert spy.calls == actions


def test_source_age_equality_valid_and_original_solve_budget_not_renewed(gate_setup):
    gate, coord, clock, spy, *_ = gate_setup
    clock.now += 500_000_000
    candidate = proposal(coord)
    assert submit(gate, candidate).proposal.command_accepted
    candidate = proposal(coord)
    clock.now += 150_000_000
    coord.receive(joystick(2, clock.now))
    coord.receive(feedback(2, clock.now))
    decision = submit(gate, candidate)
    assert "solve_deadline_expired" in decision.proposal.reason
    assert spy.calls == ["apply", "cancel", "hold"]


def test_local_hold_uses_fresh_state_without_joystick_and_stops_without_state(gate_setup):
    gate, coord, clock, spy, *_ = gate_setup
    move_candidate = proposal(coord)
    first = gate.local_hold("release")
    assert first.applied.source is None and first.applied.result.payload.command_seq > move_candidate[2].command_seq
    assert spy.held.payload.tcp_pose.position_xyz == (.4, 0, .3)
    assert submit(gate, move_candidate).disposition == "discarded"
    clock.now += NS
    second = gate.local_hold("stale feedback")
    assert second.proposal.command_accepted and spy.calls == ["cancel", "hold", "cancel", "stop"]


@pytest.mark.parametrize("failure", ["apply", "cancel", "hold", "stop"])
def test_adapter_failure_is_fault_and_never_accepts_failed_proposal(gate_setup, failure):
    gate, coord, clock, spy, *_ = gate_setup
    spy.fail.add(failure)
    if failure == "apply":
        decision = submit(gate, proposal(coord))
        assert spy.calls == ["apply", "cancel", "stop"]
    else:
        if failure == "stop": clock.now += NS
        decision = gate.local_hold("test failure")
    assert not decision.proposal.command_accepted
    assert decision.lifecycle_request == "FAULT"
    assert gate.last_eligible_result_ns is None


def test_retained_gate_trace(gate_setup, tmp_path):
    gate, coord, clock, spy, _, bundle = gate_setup
    candidate = proposal(coord)
    submit(gate, candidate)
    clock.now += 20_000_000
    submit(gate, candidate)
    gate.local_hold("release")
    trace = {"profile": "offline-candidates-v2", "configuration": to_dict(bundle.resolved.configuration),
             "evidence_level": "synthetic component / spy adapter; no observed execution",
             "dispatch": {"command_seq": candidate[2].command_seq,
                          "created_ns": candidate[2].inputs.created_ns,
                          "solve_deadline_ns": candidate[2].solve_deadline_ns,
                          "expires_ns": candidate[2].expires_ns},
             "proposal": candidate[0], "selection": candidate[1], "actions": spy.calls,
             "decisions": [{"proposal": to_dict(d.proposal), "disposition": d.disposition,
                            "actions": d.actions, "lifecycle_request": d.lifecycle_request,
                            "applied": to_dict(d.applied.result) if d.applied else None,
                            "observed_execution": d.observed_execution} for d in gate.decisions]}
    (tmp_path / "gate-trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    assert trace["actions"] == ["apply", "cancel", "hold"]


@pytest.mark.parametrize("change", ["nan", "quat", "evaluation", "solver", "frame_field"])
@pytest.mark.parametrize("obsolete", [False, True])
def test_malformed_current_result_stops_but_old_malformed_result_does_not(gate_setup, change, obsolete):
    gate, coord, clock, spy, *_ = gate_setup
    candidate = proposal(coord)
    if obsolete:
        assert submit(gate, proposal(coord)).proposal.command_accepted
    payload = candidate[0]["payload"]
    if change == "nan": payload["next_tcp_pose"]["position_xyz"][0] = float("nan")
    if change == "quat": payload["next_tcp_pose"]["quaternion_xyzw"] = [0,0,0,2]
    if change == "evaluation": payload["evaluation_valid"] = False
    if change == "solver": payload["solver_status"] = "infeasible"
    if change == "frame_field": payload["next_tcp_pose"]["frame_id"] = "camera"
    decision = submit(gate, candidate)
    assert not decision.proposal.command_accepted
    assert spy.calls == (["apply"] if obsolete else ["cancel", "hold"])
    assert decision.disposition == ("discarded" if obsolete else "rejected")


def test_registry_work_does_not_block_local_withdrawal(gate_setup, monkeypatch):
    gate, coord, clock, spy, store, _ = gate_setup
    candidate = proposal(coord)
    entered, release = Event(), Event()
    original = gate._identities

    def wait(*args):
        entered.set()
        assert release.wait(2)
        original(*args)

    monkeypatch.setattr(gate, "_identities", wait)
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = pool.submit(submit, gate, candidate)
        assert entered.wait(2)
        try:
            held = pool.submit(gate.local_hold, "local release").result(timeout=1)
            assert held.proposal.command_accepted
            coord.set_control_epoch(4)
            store.invalidate("withdraw context")
        finally:
            release.set()
        decision = pending.result(timeout=2)
    assert decision.disposition == "discarded" and spy.calls == ["cancel", "hold"]


def test_arrival_deadline_and_acceptance_expiry_are_distinct(gate_setup, monkeypatch):
    gate, coord, clock, spy, *_ = gate_setup
    candidate = proposal(coord)
    original = gate._identities

    def delay(*args):
        original(*args)
        clock.now += 190_000_000

    monkeypatch.setattr(gate, "_identities", delay)
    decision = submit(gate, candidate)
    assert decision.proposal.command_accepted
    assert gate.arrivals[0].received_ns == 10 * NS
    assert decision.proposal.checked_at == parse_record("TimePoint", stamp(clock.now), policy=NUMERIC_CANDIDATES_V2)


def test_context_cannot_change_between_final_checks_and_adapter_submission(gate_setup, monkeypatch):
    gate, coord, clock, spy, store, _ = gate_setup
    candidate = proposal(coord)
    entered, release, attempted, invalidated = Event(), Event(), Event(), Event()
    original = spy.apply

    def wait(target):
        entered.set()
        assert release.wait(2)
        original(target)

    def invalidate():
        attempted.set()
        store.invalidate("replace after submission")
        invalidated.set()

    monkeypatch.setattr(spy, "apply", wait)
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = pool.submit(submit, gate, candidate)
        assert entered.wait(2)
        changed = pool.submit(invalidate)
        assert attempted.wait(2)
        assert not invalidated.is_set()
        release.set()
        decision = pending.result(timeout=2)
        changed.result(timeout=2)
    assert decision.proposal.command_accepted and invalidated.is_set()
    assert spy.calls == ["apply"]


def test_concurrent_duplicate_submissions_only_apply_once(gate_setup):
    gate, coord, _, spy, *_ = gate_setup
    candidate = proposal(coord)
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(lambda _: submit(gate, candidate), range(2)))
    assert sum(d.proposal.command_accepted for d in results) == 1
    assert spy.calls == ["apply"]


def test_local_hold_with_continuously_advancing_clock(gate_setup):
    gate, coord, clock, spy, *_ = gate_setup

    def ticking():
        clock.now += 1
        return clock.now

    coord._clock = ticking
    decision = gate.local_hold("release")
    assert decision.proposal.command_accepted and spy.calls == ["cancel", "hold"]
    allocation = coord._local_commands[decision.applied.result.payload.command_seq]
    assert coord._stamp(decision.proposal.checked_at) >= allocation[1]


def test_failure_fallback_uses_clock_after_host_check(gate_setup, monkeypatch):
    gate, coord, clock, spy, *_ = gate_setup
    clock.now += 490_000_000
    candidate = proposal(coord)
    original = gate._checker
    calls = []

    def delayed(**kwargs):
        calls.append(1)
        if len(calls) == 1:
            clock.now += 20_000_000
            raise ValueError("injected check failure")
        return original(**kwargs)

    monkeypatch.setattr(gate, "_checker", delayed)
    decision = submit(gate, candidate)
    assert not decision.proposal.command_accepted and decision.applied is not None
    assert spy.calls == ["cancel", "stop"]
    assert decision.applied.acceptance.checked_at == parse_record("TimePoint", stamp(clock.now), policy=NUMERIC_CANDIDATES_V2)


def test_feedback_expiring_during_cancel_uses_stop(gate_setup, monkeypatch):
    gate, coord, clock, spy, *_ = gate_setup
    clock.now += 490_000_000
    original = spy.cancel

    def delayed():
        original()
        clock.now += 20_000_000

    monkeypatch.setattr(spy, "cancel", delayed)
    decision = gate.local_hold("release")
    assert decision.proposal.command_accepted and spy.calls == ["cancel", "stop"]


def test_worker_timeout_hold_at_solve_deadline_still_has_original_expiry(gate_setup):
    gate, coord, clock, spy, *_ = gate_setup
    candidate = proposal(coord, hold=True)
    candidate[0]["payload"].update(solver_status="timeout", solve_time_s=.15)
    clock.now += 150_000_000
    decision = submit(gate, candidate)
    assert decision.proposal.command_accepted and spy.calls == ["cancel", "hold"]
    assert gate.last_eligible_result_ns == clock.now


def test_callback_exception_still_reaches_stop(gate_setup, monkeypatch):
    gate, coord, _, spy, *_ = gate_setup

    def broken(**kwargs): raise RuntimeError("synthetic checker failed")

    monkeypatch.setattr(gate, "_checker", broken)
    decision = submit(gate, proposal(coord))
    assert not decision.proposal.command_accepted and spy.calls == ["cancel", "stop"]
    assert decision.applied.result.payload.status == "hold"


@pytest.mark.parametrize("hold", [False, True])
def test_result_expiring_inside_final_check_cannot_withdraw_permission(gate_setup, monkeypatch, hold):
    gate, coord, clock, spy, *_ = gate_setup
    candidate = proposal(coord, hold=hold)
    original = gate._checker

    def slow(**kwargs):
        clock.now += 200_000_000
        if not hold:
            raise ValueError("candidate failed after expiry")
        return original(**kwargs)

    monkeypatch.setattr(gate, "_checker", slow)
    decision = submit(gate, candidate)
    assert decision.disposition == "discarded" and spy.calls == []
    assert gate._enabled and gate.last_eligible_result_ns is None


@pytest.mark.parametrize("sign", [1, -1])
def test_gate_preserves_nonidentity_source_orientation_with_quaternion_sign_equivalence(gate_setup, sign):
    from scipy.spatial.transform import Rotation
    gate, coordinator, clock, spy, *_ = gate_setup
    q = Rotation.from_euler("xyz", (.1, -.2, .15)).as_quat()
    coordinator.receive(feedback(2, clock.now, rotation=tuple(q.tolist())))
    candidate = proposal(coordinator)
    candidate[0]["payload"]["next_tcp_pose"]["quaternion_xyzw"] = (sign * q).tolist()
    # New measured feedback may differ slightly. Required held orientation is
    # the pinned source orientation, while feasibility uses the current state.
    fresh_q = Rotation.from_euler("xyz", (.101, -.2, .15)).as_quat()
    coordinator.receive(feedback(3, clock.now, rotation=tuple(fresh_q.tolist())))
    decision = submit(gate, candidate)
    assert decision.disposition == "accepted_move"
    assert spy.targets[-1].measured_state.meta.sequence == 3


def test_gate_rejects_rotation_even_when_original_step_limits_allow_it(gate_setup):
    from scipy.spatial.transform import Rotation
    gate, coordinator, _, spy, *_ = gate_setup
    candidate = proposal(coordinator)
    candidate[0]["payload"]["next_tcp_pose"]["quaternion_xyzw"] = Rotation.from_euler("x", .001).as_quat().tolist()
    decision = submit(gate, candidate)
    assert decision.disposition == "rejected"
    assert "source_orientation_changed" in decision.proposal.reason
    assert "apply" not in spy.calls


def test_gate_rejects_z_motion_when_assistance_is_disabled(build):
    from ssc_core import estimate_intent, get_intent_config
    import math
    store, request, _, _ = build(change=lambda records, _: records["SessionConfig"].update(config_id="config-candidates-alpha0-v2"))
    activate(store, request)
    clock, adapter = Clock(), SpyAdapter()
    coordinator = SnapshotCoordinator(store, clock_ns=clock, clock_id=CLOCK,
        joystick_source_id="JoystickSample", robot_state_source_id="RobotState", mapping_id="normalized-xy-v1")
    coordinator.receive(joystick()); coordinator.receive(feedback())
    gate = CommandGate(coordinator, adapter, estimator_config_id="intent-candidates-alpha0-v2")
    gate.set_authority(control_epoch=0, motion_enabled=True, controller_ready=True)
    dispatch = coordinator.dispatch(coordinator.assemble())
    selected = estimate_intent(dispatch.inputs.resolved, get_intent_config("intent-candidates-alpha0-v2"),
        selection_id="alpha-zero-selection", policy=NUMERIC_CANDIDATES_V2).selection
    result = move(dispatch, result_id="alpha-zero-result")
    position = [.405, 0., .301]
    point = next(p for p in dispatch.inputs.bundle.resolved.candidates if p.candidate_id == selected.selected_candidate_id)
    distance = math.dist(position, point.position_xyz)
    result["payload"].update(selection_id=selected.selection_id,
        next_tcp_pose={"position_xyz": position, "quaternion_xyzw": [0.,0.,0.,1.]},
        candidate_evaluation={"candidate_id": point.candidate_id, "distance_m": distance,
            "raw_g": distance, "normalization_scale": .01, "penalty": (distance/.01)**2})
    selection = envelope("CandidateSelection", 1, clock.now, to_dict(selected))
    decision = gate.submit(result, selection=selection)
    assert decision.disposition == "rejected"
    assert "unassisted_height_changed" in decision.proposal.reason
    assert "apply" not in adapter.calls
