"""Installed version-pair integration: real receipts, fusion, solver, gate and plant.

Linux only. No mocked numeric success and no profile deadline extensions.
Component fault/structure coverage is joined by the acceptance-matrix generator.
"""

from dataclasses import asdict, replace
from hashlib import sha256
import json
import math
import signal
import os
import subprocess
import sys
import time

import pytest
from ssc_core import NUMERIC_OFFLINE_V1, Pose, parse_message, to_dict
from ssc_core.archive import RunArchive, environment
from ssc_runtime import LifecycleState as S, proxy_inverse
from ssc_runtime.evidence import export_run
from ssc_runtime.loop import OfflineLoop
from ssc_runtime.runner import run_normal
from ssc_runtime.scenarios import CORE_REVISION

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="real bounded workers require Linux")


@pytest.fixture
def make_loop(tmp_path):
    loops = []

    def factory(*args, **kwargs):
        value = OfflineLoop(*args, **kwargs)
        loops.append(value)
        return value

    yield factory
    for index, value in enumerate(loops):
        value.close()
        (tmp_path / f"integration-events-{index}.json").write_text(json.dumps({
            "fixture": value.fixture.variant, "scope": "real bounded computation; synthetic plant",
            "preparations": value.preparations, "solves": [asdict(s) for s in value.solves],
            "dispatches": [asdict(d) for d in value.dispatches],
            "cancellations": value.cancellations, "events": value.events,
            "callback_timings": value.callback_timings,
            "decisions": [asdict(d) for d in value.gate.decisions]}, indent=2))


@pytest.fixture
def loop(make_loop):
    with make_loop() as value:
        value.initialize()
        yield value


def move(result):
    assert result[2].disposition == "accepted_move", result[2].proposal.reason
    assert result[0].payload.status == "move"
    return result


def alter(message, **changes):
    data = to_dict(message)
    data["payload"].update(changes)
    data["meta"]["message_id"] += "-injected"
    return parse_message(data, policy=NUMERIC_OFFLINE_V1)


def held_result(loop):
    loop.input()
    dispatch = loop.dispatch()
    result, weights, _, _ = loop.wait(deliver=False)
    assert result.payload.status == "move", loop.solves[-1].diagnostic
    return result, weights, dispatch


def test_normal_loop_history_and_independent_feedback(tmp_path, make_loop):
    with make_loop() as loop:
        results = run_normal(loop)
        assert len(results) == 3
        for previous, current in zip(results, results[1:]):
            source = current[3].inputs
            assert source.previous_applied.acceptance == previous[2].proposal
            assert source.snapshot.previous_applied_command_id == previous[2].proposal.applied_command_id
            assert source.previous_applied.result.payload == previous[0].payload
        assert all(b[3].dispatched_ns - a[3].dispatched_ns == 200_000_000
                   for a, b in zip(results, results[1:]))
        apply = next(e for e in loop.plant.events if e.operation == "apply")
        assert apply.joints_before == apply.joints_after
        steps = [e for e in loop.plant.events if e.operation == "advance" and e.joints_before != e.joints_after]
        assert steps
        for e in steps:
            assert e.target is not None
            assert e.target.result.payload.result_id in {r[0].payload.result_id for r in results}
        assert loop.plant.target is None and loop.lifecycle.state is S.PAUSED
        (tmp_path / "integration-trace.json").write_text(json.dumps({
            "dispatches": [asdict(d) for d in loop.dispatches], "decisions": [asdict(d) for d in loop.gate.decisions],
            "plant": [asdict(e) for e in loop.plant.events], "lifecycle": [r.to_dict() for r in loop.lifecycle.records]}, indent=2))


@pytest.mark.parametrize("variant,dx,raw,penalty", [
    ("joystick-only", 10 * .01 / 11, None, None),
    ("positive-approach", 15 * .01 / 16, .01 / 16, 1 / 256),
    ("within-tolerance", 10 * .01 / 11, None, 0.),
])
def test_profile_golden_movement(variant, dx, raw, penalty, make_loop, tmp_path):
    with make_loop(variant) as loop:
        loop.initialize()
        result, weights, decision, dispatch = move(loop.cycle())
        assert result.payload.next_tcp_pose.position_xyz[0] - .4 == pytest.approx(dx, abs=1e-6)
        if variant == "joystick-only":
            assert weights.payload.assistance_strength == 0
            assert result.payload.constraint_evaluations  # still evaluated at alpha zero
        if raw is not None:
            assert not result.payload.constraints_satisfied and result.payload.evaluation_valid
            assert result.payload.constraint_evaluations[0].raw_g == pytest.approx(raw, abs=1e-6)
        if penalty is not None:
            assert result.payload.constraint_evaluations[0].penalty == pytest.approx(penalty, abs=1e-5)
        if variant == "within-tolerance":
            assert result.payload.constraint_evaluations[0].raw_g < 0
        (tmp_path / "integration-golden.json").write_text(json.dumps({
            "variant": variant, "expected": {"dx_m": dx, "raw_g": raw, "penalty": penalty},
            "actual": {"dx_m": result.payload.next_tcp_pose.position_xyz[0] - .4,
                "evaluations": [to_dict(v) for v in result.payload.constraint_evaluations]},
            "position_atol_m": 1e-6, "penalty_atol": 1e-5, "numerical_oracle_passed": True}, indent=2))
        # A target is a request, not a measurement at that target.
        assert loop.plant.sample().payload.tcp_pose != result.payload.next_tcp_pose


def test_current_direction_weights_and_accepted_smoothing(loop):
    right = move(loop.cycle())
    weights = {w.object_id: w.weight for w in right[1].payload.weights}
    assert weights["object-bottle-offline-v1"] == pytest.approx(3 / 4)
    loop.step(u=(1., 0.))
    left = move(loop.cycle((-1., 0.)))
    weights = {w.object_id: w.weight for w in left[1].payload.weights}
    assert weights["object-cup-offline-v1"] == pytest.approx(11 / 16)
    source = left[3].inputs.resolved.robot_state.payload.tcp_pose.position_xyz[0]
    old_source = right[3].inputs.resolved.robot_state.payload.tcp_pose.position_xyz[0]
    previous = (right[0].payload.next_tcp_pose.position_xyz[0] - old_source) / .01
    desired = sum(weights[obj] * (target - source) / .01 for obj, target in
                  (("object-bottle-offline-v1", .41), ("object-cup-offline-v1", .39)))
    expected = (-1 + .1 * previous + .5 * desired) / 1.6
    assert left[0].payload.next_tcp_pose.position_xyz[0] - source == pytest.approx(.01 * expected, abs=1e-6)
    assert expected < 0


def test_duplicate_forged_and_old_results_cannot_cancel_new_target(loop):
    first = move(loop.cycle())
    loop.step(u=(1., 0.))
    newer = move(loop.cycle())
    target = loop.plant.target
    count = len(loop.plant.events)
    for message in (first[0], newer[0], alter(first[0], command_seq=99999)):
        decision = loop.lifecycle.submit_result(message, weights=first[1] if message != newer[0] else newer[1])
        assert decision.disposition == "discarded"
        assert not decision.actions
        assert loop.plant.target is target
    assert len(loop.plant.events) == count
    assert loop.coordinator.assemble().previous_applied.acceptance == newer[2].proposal


def test_release_inflight_and_ordinary_reengagement(loop):
    # Keep a completed, undelivered real proposal in flight at the local boundary.
    result, weights, dispatch = held_result(loop)
    epoch = loop.lifecycle.control_epoch
    loop.step(release=True)
    assert loop.lifecycle.state is S.PAUSED
    assert loop.lifecycle.control_epoch == epoch + 1
    stopped = loop.plant.sample().payload.joint_positions_rad
    decision = loop.submit(result, weights)
    assert decision.disposition == "discarded" and decision.proposal.reason == "old_epoch"
    loop.step()
    assert loop.plant.sample().payload.joint_positions_rad == stopped
    bundle = loop.store.acquire()
    loop.input((0., 0.))
    assert loop.lifecycle.state is S.READY
    move(loop.cycle())
    assert loop.store.acquire() is bundle


def test_expired_result_cannot_extend_snapshot_lifetime(loop):
    result, weights, dispatch = held_result(loop)
    # Gate expiry is checked at equality before lifecycle's same-time watchdog.
    loop.clock.now = dispatch.expires_ns
    decision = loop.submit(result, weights)
    assert decision.disposition == "discarded" and decision.proposal.reason == "snapshot_expired"
    loop.lifecycle.tick()
    assert loop.lifecycle.state is S.PAUSED and loop.plant.target is None


@pytest.mark.parametrize("source", ["joystick", "state"])
def test_stale_inputs_stop_active_target(loop, source):
    move(loop.cycle())
    # Advance explicit ticks without delivering/publishing fresh selected input.
    if source == "state":
        loop.plant.set_feedback_mode("stale")
    for _ in range(26):
        loop.step(u=(1., 0.) if source == "state" else None)
    assert loop.plant.target is None and loop.lifecycle.state is S.PAUSED
    # Liveness withdraws first; freshness still prevents recovery.
    loop.input((0., 0.)) if source == "state" else loop.feedback()
    assert loop.lifecycle.state is S.PAUSED


def test_host_rejection_never_applies_proposal(make_loop):
    with make_loop(initial_joints=proxy_inverse(Pose((.499, 0., .3), (0., 0., 0., 1.)))) as loop:
        loop.initialize()
        result, weights, _ = held_result(loop)
        forged = alter(result, next_tcp_pose={"position_xyz": [.504, 0., .3], "quaternion_xyzw": [0., 0., 0., 1.]})
        decision = loop.lifecycle.submit_result(forged, weights=weights)
        assert decision.disposition == "rejected" and "host_constraint" in decision.proposal.reason
        assert loop.lifecycle.state is S.PAUSED and loop.plant.target is None
        assert not any(e.operation == "apply" for e in loop.plant.events)


def test_computation_error_is_a_hold_and_requires_repreparation(make_loop):
    joints = proxy_inverse(Pose((.405, 0., .3), (0., 0., 0., 1.)))
    with make_loop("computation-error", initial_joints=joints) as loop:
        loop.initialize()
        result, _, decision, _ = loop.cycle()
        assert result.payload.status == "hold" and result.payload.solver_status == "error"
        assert result.payload.next_tcp_pose is None and not result.payload.evaluation_valid
        assert decision.disposition == "accepted_hold"
        assert loop.lifecycle.state is S.PAUSED and not loop.lifecycle.computation_ready
        assert loop.plant.target is None
        loop.input((0., 0.))
        assert loop.lifecycle.state is S.PAUSED
        loop.prepare()
        assert loop.lifecycle.state is S.READY


def test_worker_stall_keeps_release_local(loop):
    move(loop.cycle())
    pid = loop.preparations[-1]["worker_pid"]
    os.kill(pid, signal.SIGSTOP)
    loop.input()
    loop.dispatch()
    started = time.monotonic()
    loop.step(release=True)
    assert time.monotonic() - started < .1
    assert loop.plant.target is None and loop.lifecycle.state is S.PAUSED
    assert any(t["operation"] == "on_input" and t["state_before"] == "ACTIVE" and
               t["state_after"] == "PAUSED" and t["elapsed_wall_ns"] > 0 for t in loop.callback_timings)
    late = loop.wait()
    assert late[0].payload.status == "hold"
    assert late[2].disposition == "discarded"
    assert not late[2].actions


def test_worker_stall_times_out_without_renewing_target(loop):
    move(loop.cycle())
    os.kill(loop.preparations[-1]["worker_pid"], signal.SIGSTOP)
    loop.input()
    loop.dispatch()
    result, _, decision, _ = loop.wait()
    assert result.payload.status == "hold" and result.payload.solver_status == "timeout"
    assert result.payload.next_tcp_pose is None
    assert loop.plant.target is None and loop.lifecycle.state is S.PAUSED
    assert any("cancel" in d.actions for d in loop.gate.decisions)


def test_context_reinitialization_and_explicit_fault_rearm(loop):
    move(loop.cycle())
    old = loop.store.acquire()
    loop.lifecycle.invalidate_context("scene moved")
    loop.input((0., 0.))
    loop.input()
    assert loop.lifecycle.state is S.PAUSED and loop.store.acquire() is None
    loop.initialize()
    assert loop.store.acquire() is not old
    move(loop.cycle())
    loop.plant.inject_controller_fault("integration fault")
    loop.feedback()
    assert loop.lifecycle.state is S.FAULT and loop.plant.target is None
    assert any(t["operation"] == "controller_status" and t["state_before"] == "ACTIVE" and
               t["state_after"] == "FAULT" and t["elapsed_wall_ns"] > 0 for t in loop.callback_timings)
    loop.plant.clear_fault()
    loop.feedback()
    loop.input((0., 0.))
    loop.input()
    assert loop.lifecycle.state is S.FAULT
    loop.lifecycle.rearm("explicit simulated repair")
    assert loop.lifecycle.state is S.UNINITIALIZED
    loop.initialize()
    move(loop.cycle())


def test_real_run_archive_and_fresh_process_replay(tmp_path, make_loop):
    # Test-only installation identifiers; CLI acceptance separately checks real
    # wheel bytes/SHAs and records both exact revisions in its manifest.
    software = {"server": {"revision": CORE_REVISION, "package_version": environment()["packages"]["ssc-core"],
                           "wheel_sha256": "0" * 64},
                "ros2": {"revision": "0" * 40, "package_version": "0.6.0", "wheel_sha256": "0" * 64}}
    with make_loop() as loop:
        run_normal(loop)
        loop.close()
        evidence = export_run(loop, tmp_path / "run.json", software)
    loaded = RunArchive.read(evidence["path"], expected_sha256=evidence["sha256"])
    envelope = json.loads(loaded.to_bytes())
    assert evidence["payload_sha256"] == envelope["sha256"]
    assert evidence["payload_sha256"] != evidence["sha256"]
    assert evidence["accepted_moves"] == 3
    replay = subprocess.run([sys.executable, "-I", "-m", "ssc_core.replay", evidence["path"],
                             "--sha256", evidence["sha256"]], capture_output=True, text=True, timeout=60)
    (tmp_path / "integration-replay.json").write_text(replay.stdout)
    assert replay.returncode == 0, replay.stdout + replay.stderr
    assert loaded.registry().resolve_snapshot(loop.dispatches[-1].inputs.snapshot.snapshot_id)


@pytest.mark.parametrize("release_kwargs", [{"u": (0., 0.)}, {"release": True}])
def test_neutral_step_cancels_before_plant_advancement(loop, release_kwargs):
    move(loop.cycle())
    joints = loop.plant.sample().payload.joint_positions_rad
    epoch = loop.lifecycle.control_epoch
    loop.step(**release_kwargs)
    assert loop.plant.sample().payload.joint_positions_rad == joints
    assert loop.plant.target is None and loop.lifecycle.state is S.PAUSED
    assert loop.lifecycle.control_epoch == epoch + 1


def test_new_loop_has_fresh_session_and_rejects_prior_session_work(make_loop):
    with make_loop() as previous:
        previous.initialize()
        old = move(previous.cycle())
    with make_loop() as current:
        assert current.session_id != previous.session_id
        current.initialize()
        fresh = move(current.cycle())
        target = current.plant.target
        decision = current.lifecycle.submit_result(old[0], weights=old[1])
        assert decision.disposition == "discarded" and not decision.actions
        assert current.plant.target is target


@pytest.mark.parametrize("variant,start,max_x", [
    ("workspace-face", (.499, 0., .3), .5),
    ("excluded-sphere", (.42, .07, .3), .4299),
])
def test_soft_relation_cannot_relax_real_host_limits(make_loop, variant, start, max_x):
    with make_loop(variant, initial_joints=proxy_inverse(Pose(start, (0., 0., 0., 1.)))) as loop:
        loop.initialize()
        result, _, decision, _ = move(loop.cycle())
        target = result.payload.next_tcp_pose.position_xyz
        assert start[0] < target[0] <= max_x + 2e-9
        assert result.payload.evaluation_valid and not result.payload.constraints_satisfied
        assert result.payload.constraint_evaluations[0].raw_g > 0
        assert loop.solves[-1].host_margins
        assert min(margin for _, margin in loop.solves[-1].host_margins) >= -1e-8
        loop.step(u=(1., 0.))
        actual = loop.plant.sample().payload.tcp_pose.position_xyz
        assert start[0] < actual[0] <= target[0] + 2e-9
