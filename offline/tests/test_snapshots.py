from concurrent.futures import ThreadPoolExecutor
from dataclasses import FrozenInstanceError, replace
import json
from threading import Barrier

import pytest

from ssc_core import GeometryRef, NUMERIC_OFFLINE_V1, ValidationError, parse_message, to_dict
from ssc_runtime import SnapshotCoordinator, SnapshotError

NS = 1_000_000_000
CLOCK = "synthetic-monotonic-epoch-1"


class Clock:
    now = 10 * NS

    def __call__(self):
        return self.now


def stamp(ns, clock=CLOCK):
    return {"clock_id": clock, "sec": ns // NS, "nanosec": ns % NS}


def envelope(kind, sequence, ns, payload, *, source=None):
    return {"message_type": kind, "meta": {
        "schema_version": "1.0", "session_id": "session-offline-v1",
        "message_id": f"{kind}-{sequence}", "source_id": source or kind,
        "sequence": sequence, "produced_at": stamp(ns),
    }, "payload": payload}


def joystick(sequence=1, ns=10 * NS, **changes):
    payload = {"sampled_at": stamp(ns), "mapping_id": "normalized-xy-v1",
               "u_xy": [1, 0], "enabled": True}
    payload.update(changes)
    return envelope("JoystickSample", sequence, ns, payload)


def state(sequence=1, ns=10 * NS, **changes):
    payload = {"sampled_at": stamp(ns), "frame_id": "synthetic_base", "tcp_frame": "synthetic_tcp",
               "tcp_pose": {"position_xyz": [0, 0, .2], "quaternion_xyzw": [0, 0, 0, 1]},
               "joint_names": [f"q{i}" for i in range(6)], "joint_positions_rad": [0] * 6,
               "state_source": "measured_feedback"}
    payload.update(changes)
    return envelope("RobotState", sequence, ns, payload)


def activate(store, request):
    outcome = store.publish(store.prepare(store.begin(request)))
    assert outcome.status == "activated"
    return outcome.bundle


@pytest.fixture
def setup(build):
    store, request, _, _ = build()
    activate(store, request)
    clock = Clock()
    coordinator = SnapshotCoordinator(store, clock_ns=clock, clock_id=CLOCK,
                                      joystick_source_id="JoystickSample", robot_state_source_id="RobotState",
                                      mapping_id="normalized-xy-v1", control_epoch=3)
    return coordinator, clock, store, request


def feed(coordinator, *, seq=1, ns=10 * NS):
    coordinator.receive(joystick(seq, ns))
    coordinator.receive(state(seq, ns))


def move(dispatch, *, result_id="result-1", produced_ns=10 * NS):
    source = dispatch.inputs
    payload = {"result_id": result_id, "snapshot_id": source.snapshot.snapshot_id,
               "weights_id": "weights-1", "control_epoch": source.snapshot.control_epoch,
               "command_seq": dispatch.command_seq, "source_joystick_seq": source.joystick.message.meta.sequence,
               "source_robot_state_seq": source.robot_state.message.meta.sequence,
               "status": "move", "evaluation_valid": True, "solver_status": "solved",
               "constraints_satisfied": False, "solve_time_s": .01, "constraint_evaluations": [],
               "next_tcp_pose": {"position_xyz": [.001, 0, .2], "quaternion_xyzw": [0, 0, 0, 1]},
               "horizon_s": source.snapshot.dt_s}
    message = envelope("ControlResult", dispatch.command_seq, produced_ns, payload)
    message["meta"]["message_id"] = result_id + "-message"
    return message


def acceptance(message, *, ns=10 * NS, accepted=True, applied="applied-1"):
    return {"result_id": message["payload"]["result_id"], "command_accepted": accepted,
            "reason": "trusted synthetic adapter decision", "checked_at": stamp(ns),
            "applied_command_id": applied if accepted else None}


def test_initial_snapshot_exact_graph_and_detached_data(setup):
    coordinator, clock, store, request = setup
    original = joystick()
    coordinator.receive(original)
    coordinator.receive(state())
    frozen = coordinator.assemble()
    snapshot = frozen.snapshot
    assert snapshot.snapshot_seq == 1 and snapshot.control_epoch == 3
    assert snapshot.dt_s == .2 and snapshot.joystick_history_refs == ()
    assert snapshot.previous_applied_command_id is None
    assert frozen.bundle is store.acquire()
    assert frozen.resolved.bundle is frozen.bundle.resolved
    assert snapshot.program_refs == request.program_refs
    assert snapshot.execution_binding_refs == request.execution_binding_refs
    assert frozen.expires_ns == 10_200_000_000 and frozen.created_ns == clock.now
    assert "command_seq" not in to_dict(snapshot)
    original["payload"]["u_xy"][0] = -1
    assert frozen.resolved.joystick.payload.u_xy == (1, 0)
    with pytest.raises(FrozenInstanceError):
        frozen.expires_ns += NS
    with pytest.raises(TypeError):
        frozen.resolved.bundle.programs[0].keypoints[0][0] = 9
    with store.artifacts() as (registry, _):
        assert registry.resolve_snapshot(snapshot.snapshot_id) == frozen.resolved


@pytest.mark.parametrize("missing", ["both", "state", "joystick"])
def test_missing_samples_do_not_become_zero(setup, missing):
    coordinator, *_ = setup
    if missing == "state":
        coordinator.receive(joystick())
    elif missing == "joystick":
        coordinator.receive(state())
    with pytest.raises(SnapshotError, match="missing_input"):
        coordinator.assemble()


@pytest.mark.parametrize("factory", [joystick, state])
def test_stale_at_receipt_and_threshold_equality(setup, factory):
    coordinator, clock, *_ = setup
    feed(coordinator)
    clock.now += 500_000_000
    assert coordinator.assemble().motion_eligible
    clock.now += 1
    # Fresh metadata/receipt cannot repair old acquisition.
    data = factory(2)
    data["meta"]["produced_at"] = stamp(clock.now)
    timing = coordinator.receive(data)
    assert timing.sampled_ns == 10 * NS and timing.received_ns == clock.now
    with pytest.raises(SnapshotError, match="stale_input"):
        coordinator.assemble()


@pytest.mark.parametrize("factory", [joystick, state])
@pytest.mark.parametrize("change,code", [
    ("foreign_clock", "unverifiable_clock"), ("future", "future_sample"),
    ("session", "session_mismatch"), ("source", "input_source"),
    ("replay", "replayed_input"), ("reversed", "reversed_sample_time"),
])
def test_input_provenance_rejections_preserve_current(setup, factory, change, code):
    coordinator, *_ = setup
    feed(coordinator)
    data = factory(2)
    if change == "foreign_clock": data["payload"]["sampled_at"] = stamp(10 * NS, "workstation-offset")
    if change == "future": data["payload"]["sampled_at"] = stamp(10 * NS + 1)
    if change == "session": data["meta"]["session_id"] = "previous-session"
    if change == "source": data["meta"]["source_id"] = "unconfigured-source"
    if change == "replay": data["meta"]["sequence"] = 1
    if change == "reversed": data["payload"]["sampled_at"] = stamp(10 * NS - 1)
    with pytest.raises(SnapshotError, match=code):
        coordinator.receive(data)
    assert coordinator.assemble().snapshot.joystick_ref == "JoystickSample-1"
    assert coordinator.assemble().snapshot.robot_state_ref == "RobotState-1"


def test_foreign_production_clock_is_not_subtracted(setup):
    coordinator, clock, *_ = setup
    for data in (joystick(), state()):
        data["meta"]["produced_at"] = stamp(900_000 * NS, "remote-wall-clock")
        coordinator.receive(data)
    frozen = coordinator.assemble()
    dispatch = coordinator.dispatch(frozen)
    clock.now = frozen.expires_ns
    result = move(dispatch, produced_ns=clock.now + 100 * NS)
    result["meta"]["produced_at"]["clock_id"] = "another-remote-clock"
    arrival = coordinator.observe_result(result)
    assert not arrival.before_expiry and not arrival.before_solve_deadline
    assert coordinator.lookup_dispatch(frozen.snapshot.snapshot_id) is dispatch


@pytest.mark.parametrize("changes,code", [
    ({"frame_id": "camera"}, "frame_mismatch"), ({"tcp_frame": "flange"}, "frame_mismatch"),
])
def test_incompatible_state_blocks_snapshot(setup, changes, code):
    coordinator, *_ = setup
    coordinator.receive(joystick())
    coordinator.receive(state(**changes))
    with pytest.raises(SnapshotError, match=code): coordinator.assemble()


def test_bad_mapping_and_commanded_feedback_are_rejected(setup):
    coordinator, *_ = setup
    with pytest.raises(SnapshotError, match="mapping_mismatch"):
        coordinator.receive(joystick(mapping_id="raw-device-axes"))
    with pytest.raises(ValidationError):
        coordinator.receive(state(state_source="commanded_pose"))
    bad = parse_message(joystick(), policy=NUMERIC_OFFLINE_V1)
    with pytest.raises(ValidationError):
        coordinator.receive(replace(bad, payload=replace(bad.payload, u_xy=(float("nan"), 0))))


def test_history_latest_32_order_equal_stamps_and_window(setup):
    coordinator, clock, *_ = setup
    coordinator.receive(state())
    for seq in range(1, 42):
        coordinator.receive(joystick(seq, enabled=seq % 2 == 0))
    frozen = coordinator.assemble()
    assert [x.message.meta.sequence for x in frozen.history] == list(range(9, 41))
    assert frozen.snapshot.joystick_history_refs == tuple(f"JoystickSample-{i}" for i in range(9, 41))
    assert all(item.sampled_ns == 10 * NS for item in frozen.history)
    clock.now += NS
    feed(coordinator, seq=42, ns=clock.now)
    boundary = coordinator.assemble()
    assert len(boundary.history) == 32  # equality at window boundary is retained
    clock.now += 1
    after = coordinator.assemble()
    assert after.history == ()
    assert len(frozen.history) == 32


@pytest.mark.parametrize("changes", [{"enabled": False}, {"u_xy": [0, 0]}])
def test_neutral_disabled_current_never_borrow_history_motion(setup, changes):
    coordinator, *_ = setup
    feed(coordinator)
    coordinator.receive(joystick(2, **changes))
    frozen = coordinator.assemble()
    assert len(frozen.history) == 1 and not frozen.motion_eligible
    with pytest.raises(SnapshotError, match="motion_disabled"): coordinator.dispatch(frozen)


def test_delayed_dispatch_retry_original_expiry_and_independent_sequences(setup):
    coordinator, clock, *_ = setup
    feed(coordinator)
    unused = coordinator.assemble()
    frozen = coordinator.assemble()
    assert coordinator.allocate_local_command_sequence() == 1
    assert coordinator.allocate_local_command_sequence() == 2
    clock.now += 100_000_000
    dispatch = coordinator.dispatch(frozen)
    assert dispatch.command_seq == 3 != frozen.snapshot.snapshot_seq
    assert dispatch.expires_ns == unused.expires_ns == 10_200_000_000
    assert dispatch.solve_deadline_ns == dispatch.expires_ns
    clock.now += 20_000_000
    assert coordinator.dispatch(frozen) is dispatch
    clock.now = dispatch.expires_ns
    with pytest.raises(SnapshotError, match="snapshot_expired"): coordinator.dispatch(frozen)


def test_retry_cannot_restart_solve_and_snapshot_cannot_be_forged(setup):
    coordinator, clock, *_ = setup
    feed(coordinator)
    frozen = coordinator.assemble()
    with pytest.raises(SnapshotError, match="foreign_snapshot"):
        coordinator.dispatch(replace(frozen, expires_ns=frozen.expires_ns + NS))
    dispatch = coordinator.dispatch(frozen)
    clock.now = dispatch.solve_deadline_ns
    with pytest.raises(SnapshotError, match="solve_expired"): coordinator.dispatch(frozen)
    assert clock.now < frozen.expires_ns


def test_new_input_does_not_change_epoch_or_dispatched_inputs(setup):
    coordinator, clock, *_ = setup
    feed(coordinator)
    frozen = coordinator.assemble()
    dispatch = coordinator.dispatch(frozen)
    clock.now += 20_000_000
    coordinator.receive(joystick(2, clock.now, u_xy=[-1, 0]))
    assert coordinator.control_epoch == 3
    assert coordinator.dispatch(frozen) is dispatch
    assert dispatch.inputs.resolved.joystick.payload.u_xy == (1, 0)
    assert coordinator.assemble().resolved.joystick.payload.u_xy == (-1, 0)
    coordinator.receive(joystick(3, clock.now, enabled=False))
    with pytest.raises(SnapshotError, match="motion_disabled"): coordinator.dispatch(frozen)


def test_old_context_and_epoch_retain_evidence_but_cannot_dispatch(setup):
    coordinator, _, store, request = setup
    feed(coordinator)
    old = coordinator.assemble()
    old_dispatch = coordinator.dispatch(old)
    activate(store, request)
    new = coordinator.assemble()
    assert old.bundle.activation_id != new.bundle.activation_id
    assert old_dispatch.inputs is old
    with pytest.raises(SnapshotError, match="superseded_bundle"): coordinator.dispatch(old)
    coordinator.set_control_epoch(4)
    with pytest.raises(SnapshotError, match="old_epoch"): coordinator.dispatch(new)
    assert coordinator.assemble().snapshot.control_epoch == 4
    with pytest.raises(SnapshotError, match="old_epoch"): coordinator.set_control_epoch(4)
    store.invalidate("scene moved")
    with pytest.raises(SnapshotError, match="no_active_bundle"): coordinator.assemble()


def test_dispatched_arrays_survive_actual_geometry_revision(setup):
    coordinator, _, store, request = setup
    feed(coordinator)
    old = coordinator.assemble()
    dispatch = coordinator.dispatch(old)
    before = dispatch.inputs.resolved.bundle.programs[0].keypoints
    geometry = to_dict(old.resolved.bundle.geometry)
    geometry.update(geometry_id="geometry-revision-2", revision=2)
    geometry["keypoints"][0]["position_xyz"][0] += .01
    with store.artifacts() as (registry, _):
        registry.register_record("SceneGeometry", geometry, session_id=store.session_id)
    activate(store, replace(request, geometry_ref=GeometryRef(geometry["geometry_id"], 2)))
    new = coordinator.assemble()
    assert new.resolved.bundle.programs[0].keypoints != before
    assert dispatch.inputs.resolved.bundle.programs[0].keypoints == before
    assert old.snapshot.geometry_ref.revision == 1 and new.snapshot.geometry_ref.revision == 2
    with pytest.raises(SnapshotError, match="superseded_bundle"): coordinator.dispatch(old)


def test_current_input_accessor_preserves_release_and_does_not_allocate(setup):
    coordinator, clock, *_ = setup
    feed(coordinator)
    frozen = coordinator.assemble()
    coordinator.receive(joystick(2, enabled=False))
    live_joystick, live_state = coordinator.current_inputs()
    assert not live_joystick.message.payload.enabled
    assert live_state.message.meta.sequence == 1
    assert frozen.resolved.joystick.payload.enabled
    assert coordinator.assemble().snapshot.snapshot_seq == 2
    clock.now += 500_000_001
    with pytest.raises(SnapshotError, match="stale_input"): coordinator.current_inputs()


def test_dispatch_checks_pinned_acquisition_age_even_with_new_live_input(setup):
    coordinator, clock, *_ = setup
    # Snapshot can be fresh enough at creation but pinned input can age out
    # before its own command expiry, even when replacement input is fresh.
    feed(coordinator, ns=9_550_000_000)
    frozen = coordinator.assemble()
    clock.now += 60_000_000
    feed(coordinator, seq=2, ns=clock.now)
    assert coordinator.current_inputs()[0].sampled_ns == clock.now
    with pytest.raises(SnapshotError, match="stale_input"): coordinator.dispatch(frozen)


@pytest.mark.parametrize("field,value", [("command_seq", 999), ("control_epoch", 4),
    ("source_joystick_seq", 999), ("source_robot_state_seq", 999), ("horizon_s", .1)])
def test_result_must_echo_host_dispatch(setup, field, value):
    coordinator, *_ = setup
    feed(coordinator)
    result = move(coordinator.dispatch(coordinator.assemble()))
    result["payload"][field] = value
    with pytest.raises(SnapshotError, match="result_mismatch"): coordinator.observe_result(result)


def test_only_accepted_adapter_decision_populates_previous(setup):
    coordinator, clock, *_ = setup
    feed(coordinator)
    first = coordinator.assemble()
    result = move(coordinator.dispatch(first))
    coordinator.observe_result(result)
    assert coordinator.assemble().snapshot.previous_applied_command_id is None
    assert coordinator.record_acceptance(result, acceptance(result, accepted=False)) is None
    assert coordinator.assemble().snapshot.previous_applied_command_id is None
    accepted = coordinator.record_acceptance(result, acceptance(result))
    frozen = coordinator.assemble()
    assert frozen.snapshot.previous_applied_command_id == "applied-1"
    assert frozen.previous_applied is accepted and accepted.source is first
    assert coordinator.lookup_applied("applied-1") is accepted
    with pytest.raises(SnapshotError, match="duplicate_acceptance"):
        coordinator.record_acceptance(result, acceptance(result))
    coordinator.set_control_epoch(4)
    assert coordinator.assemble().previous_applied.result.payload.control_epoch == 3
    assert clock.now == 10 * NS  # no physical feedback/advance is implied


def test_latest_accepted_hold_is_retained_not_skipped(setup):
    coordinator, *_ = setup
    feed(coordinator)
    result = move(coordinator.dispatch(coordinator.assemble()))
    coordinator.record_acceptance(result, acceptance(result))
    seq = coordinator.allocate_local_command_sequence()
    hold = envelope("ControlResult", seq, 10 * NS, {
        "result_id": "hold-1", "snapshot_id": None, "weights_id": None,
        "control_epoch": 3, "command_seq": seq, "source_joystick_seq": None,
        "source_robot_state_seq": None, "status": "hold", "evaluation_valid": False,
        "solver_status": "not_run", "constraints_satisfied": None, "solve_time_s": 0,
        "constraint_evaluations": [], "reason": "release"})
    coordinator.record_acceptance(hold, acceptance(hold, applied="applied-hold"))
    frozen = coordinator.assemble()
    assert frozen.snapshot.previous_applied_command_id == "applied-hold"
    assert frozen.previous_applied.result.payload.status == "hold"
    assert frozen.previous_applied.source is None


@pytest.mark.parametrize("case,code", [("pair", "acceptance_mismatch"),
    ("result_id", "acceptance_mismatch"), ("session", "session_mismatch"),
    ("future", "future_acceptance"), ("clock", "unverifiable_clock"),
    ("expired", "acceptance_expired")])
def test_inconsistent_acceptance_never_populates_history(setup, case, code):
    coordinator, clock, *_ = setup
    feed(coordinator)
    dispatch = coordinator.dispatch(coordinator.assemble())
    result = move(dispatch)
    record = acceptance(result)
    if case == "pair": record["applied_command_id"] = None
    if case == "result_id": record["result_id"] = "foreign-result"
    if case == "session": result["meta"]["session_id"] = "previous-session"
    if case == "future": record["checked_at"] = stamp(clock.now + 1)
    if case == "clock": record["checked_at"] = stamp(clock.now, "remote-clock")
    if case == "expired":
        clock.now = dispatch.expires_ns
        record["checked_at"] = stamp(clock.now)
    with pytest.raises(SnapshotError, match=code): coordinator.record_acceptance(result, record)
    assert coordinator.assemble().snapshot.previous_applied_command_id is None


def test_out_of_order_acceptance_cannot_replace_newer(setup):
    coordinator, *_ = setup
    feed(coordinator)
    old = move(coordinator.dispatch(coordinator.assemble()), result_id="old")
    new = move(coordinator.dispatch(coordinator.assemble()), result_id="new")
    coordinator.record_acceptance(new, acceptance(new, applied="new-applied"))
    with pytest.raises(SnapshotError, match="out_of_order_acceptance"):
        coordinator.record_acceptance(old, acceptance(old, applied="old-applied"))
    assert coordinator.assemble().snapshot.previous_applied_command_id == "new-applied"


def test_clock_regression_latches_failure(setup):
    coordinator, clock, *_ = setup
    feed(coordinator)
    clock.now -= 1
    with pytest.raises(SnapshotError, match="invalid_clock"): coordinator.assemble()
    clock.now += 2
    with pytest.raises(SnapshotError, match="invalid_clock"): coordinator.assemble()


def test_transport_allowance_and_budget_fit(setup):
    _, clock, store, _ = setup
    def make(allowance):
        return SnapshotCoordinator(store, clock_ns=clock, clock_id=CLOCK,
                                   joystick_source_id="JoystickSample", robot_state_source_id="RobotState",
                                   mapping_id="normalized-xy-v1", transport_allowance_ns=allowance)
    good = make(50_000_000)
    feed(good)
    frozen = good.assemble()
    clock.now += 100_000_000
    dispatch = good.dispatch(frozen)
    assert dispatch.solve_deadline_ns == frozen.expires_ns - 50_000_000
    bad = make(50_000_001)
    feed(bad)
    with pytest.raises(SnapshotError, match="budget_mismatch"): bad.assemble()


def test_concurrent_retry_allocates_one_sequence(setup):
    coordinator, *_ = setup
    feed(coordinator)
    frozen = coordinator.assemble()
    barrier = Barrier(4)
    def dispatch():
        barrier.wait(timeout=5)
        return coordinator.dispatch(frozen)
    with ThreadPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(dispatch) for _ in range(4)]
        records = [f.result(timeout=5) for f in futures]
    assert all(item is records[0] for item in records)
    assert coordinator.allocate_local_command_sequence() == 2


def test_retained_trace_contains_resolved_inputs_and_original_deadlines(setup, tmp_path):
    coordinator, clock, *_ = setup
    feed(coordinator)
    frozen = coordinator.assemble()
    clock.now += 60_000_000
    dispatch = coordinator.dispatch(frozen)
    clock.now = frozen.expires_ns
    arrival = coordinator.observe_result(move(dispatch))
    trace = {"profile": "offline-reference-v1", "configuration": to_dict(frozen.bundle.resolved.configuration),
             "snapshot": to_dict(frozen.snapshot), "clock_id": frozen.clock_id,
             "joystick": to_dict(frozen.resolved.joystick), "state": to_dict(frozen.resolved.robot_state),
             "program_keypoints": [p.keypoints for p in frozen.resolved.bundle.programs],
             "creation_ns": frozen.created_ns, "expiry_ns": frozen.expires_ns,
             "command_seq": dispatch.command_seq, "dispatch_ns": dispatch.dispatched_ns,
             "solve_deadline_ns": dispatch.solve_deadline_ns,
             "arrival_ns": arrival.received_ns, "before_expiry": arrival.before_expiry,
             "evidence_level": "synthetic component timing; no gate, plant or hardware"}
    (tmp_path / "snapshot-trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    assert trace["expiry_ns"] - trace["creation_ns"] == 200_000_000
    assert not trace["before_expiry"]
