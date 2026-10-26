"""P1-13 lifecycle, withdrawal, watchdog and recovery tests.

All evidence is software simulation with a controllable integer-nanosecond clock
and recording adapters. Numerical receipt generation is stubbed by the shared
fixture; registry, host trust, snapshot, gate and lifecycle logic are real.
"""

from dataclasses import replace
import json

import pytest
from ssc_core import Pose, to_dict
from ssc_runtime import (CommandGate, LifecycleCoordinator, LifecycleError, SessionRuntime,
                         SnapshotCoordinator, proxy_inverse)
from ssc_runtime import LifecycleState as S

NS = 1_000_000_000
MS = 1_000_000
T0 = 10 * NS
SESSION = "session-offline-v1"


class Clock:
    def __init__(self):
        self.now = T0

    def __call__(self):
        return self.now


class SpyAdapter:
    """Recording CommandAdapter with injectable submission failures."""

    def __init__(self):
        self.calls, self.targets, self.held, self.fail = [], [], [], set()

    def _call(self, name):
        self.calls.append(name)
        if name in self.fail:
            raise RuntimeError(name + " failed")

    def apply(self, target):
        self._call("apply")
        self.targets.append(target)

    def cancel(self):
        self._call("cancel")

    def hold(self, measured_state):
        self._call("hold")
        self.held.append(measured_state)

    def stop(self):
        self._call("stop")


class SpyCanceller:
    def __init__(self):
        self.calls, self.fail = [], False

    def cancel(self, control_epoch, reason):
        self.calls.append((control_epoch, reason))
        if self.fail:
            raise RuntimeError("worker unreachable")


class Harness:
    def __init__(self, build):
        self.build, self.clock = build, Clock()
        self.clock_id = "synthetic-monotonic-epoch-1"
        self.adapter, self.canceller = SpyAdapter(), SpyCanceller()
        self.requests, self.counters = {}, {}
        self.lc = LifecycleCoordinator(self.factory, session_id=SESSION, worker_canceller=self.canceller)

    # Trusted host session factory: fresh registry, coordinator and gate.
    def factory(self, session_id):
        store, request, _, _ = self.build(session_id=session_id)
        self.requests[session_id] = request
        self.counters[session_id] = {"JoystickSample": 0, "RobotState": 0}
        coordinator = SnapshotCoordinator(
            store, clock_ns=self.clock, clock_id=self.clock_id, joystick_source_id="JoystickSample",
            robot_state_source_id="RobotState", mapping_id="normalized-xy-v1")
        return SessionRuntime(store, coordinator, CommandGate(coordinator, self.adapter))

    @property
    def session(self):
        return self.lc.session_id

    @property
    def gate(self):
        return self.lc.runtime.gate

    @property
    def coordinator(self):
        return self.lc.runtime.coordinator

    @property
    def store(self):
        return self.lc.runtime.store

    def stamp(self, ns, clock_id=None):
        return {"clock_id": clock_id or self.clock_id, "sec": ns // NS, "nanosec": ns % NS}

    def envelope(self, kind, payload, *, session=None, sequence=None):
        session = session or self.session
        if sequence is None:
            self.counters[session][kind] += 1
            sequence = self.counters[session][kind]
        return {"message_type": kind, "meta": {
            "schema_version": "1.0", "session_id": session, "message_id": f"{kind}-{sequence}",
            "source_id": kind, "sequence": sequence, "produced_at": self.stamp(self.clock.now)},
            "payload": payload}

    def joystick(self, u=(1., 0.), enabled=True, *, at=None, clock_id=None, **kw):
        at = self.clock.now if at is None else at
        return self.envelope("JoystickSample", {"sampled_at": self.stamp(at, clock_id),
                             "mapping_id": "normalized-xy-v1", "u_xy": list(u), "enabled": enabled}, **kw)

    def state(self, position=(.4, 0., .3), *, at=None, consistent=True, clock_id=None, **kw):
        at = self.clock.now if at is None else at
        pose = Pose(tuple(position), (0., 0., 0., 1.))
        joints = list(proxy_inverse(pose))
        if not consistent:
            joints[0] += .05  # measured joints disagree with measured TCP
        return self.envelope("RobotState", {
            "sampled_at": self.stamp(at, clock_id), "frame_id": "synthetic_base", "tcp_frame": "synthetic_tcp",
            "tcp_pose": to_dict(pose), "joint_names": [f"q{i}" for i in range(1, 7)],
            "joint_positions_rad": joints, "state_source": "fk_from_measured_joints"}, **kw)

    def push(self, u=(1., 0.), enabled=True, **kw):
        return self.lc.on_input(self.joystick(u, enabled, **kw))

    def neutral(self, **kw):
        return self.push((0., 0.), **kw)

    def feed(self, position=(.4, 0., .3), **kw):
        return self.lc.on_input(self.state(position, **kw))

    def advance(self, ms):
        self.clock.now += int(round(ms * MS))

    def fresh(self, u=(1., 0.)):
        """Fresh feedback then joystick at the current tick (profile event order)."""
        self.feed()
        self.push(u)

    def ready(self):
        self.feed()
        self.neutral()
        outcome = self.lc.initialize(self.requests[self.session])
        assert outcome.status == "activated", outcome
        self.lc.controller_status(ready=True)
        self.lc.computation_status(ready=True, reason="solver prepared with motion disabled")
        assert self.lc.state is S.READY
        return outcome.bundle

    def active(self):
        bundle = self.ready()
        self.push()
        assert self.lc.state is S.ACTIVE
        return bundle

    def result(self, dispatch, *, position=(.405, 0., .3), hold=False, tag="r"):
        source, seq = dispatch.inputs, dispatch.command_seq
        bindings = source.bundle.resolved.execution_bindings
        weights_id = f"weights-{seq}-{tag}"
        payload = {
            "result_id": f"proposal-{seq}-{tag}", "snapshot_id": source.snapshot.snapshot_id,
            "weights_id": weights_id, "control_epoch": source.snapshot.control_epoch, "command_seq": seq,
            "source_joystick_seq": source.joystick.message.meta.sequence,
            "source_robot_state_seq": source.robot_state.message.meta.sequence,
            "status": "move", "evaluation_valid": True, "solver_status": "solved",
            "constraints_satisfied": False, "solve_time_s": .01,
            "constraint_evaluations": [{"program_id": b.program_id, "raw_g": .001,
                                        "normalization_scale": b.normalization_scale,
                                        "penalty": (.001 / b.normalization_scale) ** 2} for b in bindings],
            "next_tcp_pose": {"position_xyz": list(position), "quaternion_xyzw": [0, 0, 0, 1]},
            "horizon_s": source.snapshot.dt_s}
        if hold:
            payload.update(status="hold", reason="worker evaluation failure", evaluation_valid=False,
                           solver_status="error", constraints_satisfied=None)
            payload.pop("next_tcp_pose")
            payload.pop("horizon_s")
        meta = {"schema_version": "1.0", "session_id": source.session_id, "sequence": seq,
                "produced_at": self.stamp(self.clock.now)}
        candidates = source.bundle.resolved.candidates
        result = {"message_type": "ControlResult", "payload": payload,
                  "meta": {**meta, "message_id": f"ControlResult-{seq}-{tag}", "source_id": "optimizer"}}
        weights = {"message_type": "ObjectWeights", "meta": {
            **meta, "message_id": f"ObjectWeights-{seq}-{tag}", "source_id": "estimator"}, "payload": {
            "weights_id": weights_id, "snapshot_id": source.snapshot.snapshot_id,
            "estimator_config_id": "intent-linear-v1",
            "weights": [{"object_id": c.object_id, "weight": 1 / len(candidates)} for c in candidates],
            "assistance_strength": .5}}
        return result, weights

    def submit(self, pair):
        return self.lc.submit_result(pair[0], weights=pair[1])

    def withdrawals(self):
        return [r for r in self.lc.records if r.kind == "withdrawal"]

    def transitions(self):
        return [(r.state_before, r.state_after, r.event) for r in self.lc.records
                if r.state_before is not r.state_after]


@pytest.fixture
def h(build):
    return Harness(build)


def assert_withdrawal(record, *, before, after, epoch, action):
    assert (record.state_before, record.state_after) == (before, after)
    assert record.epoch_after == record.epoch_before + 1 == epoch + 1
    assert {"withdraw_authority", "increment_epoch", "cancel_worker"} <= set(record.actions)
    actions = list(record.actions)
    assert actions.index("cancel") < actions.index(action)
    assert record.observed_execution is None


# --------------------------------------------------------------------- AC1, C02
@pytest.mark.parametrize("enabled,u,reason", [(True, (0., 0.), "joystick_released"),
                                              (False, (1., 0.), "joystick_disabled")])
def test_release_during_stalled_solve_is_local_and_rejects_late_result(h, enabled, u, reason):
    h.active()
    stalled = h.lc.dispatch_cycle()  # the worker never answers before release
    h.advance(40)
    h.feed()
    h.push(u, enabled)
    assert h.lc.state is S.PAUSED and h.lc.control_epoch == 1
    record = h.withdrawals()[-1]
    assert record.event == "release" and record.reason == reason
    assert list(record.actions) == ["withdraw_authority", "increment_epoch", "allocate_local_hold",
                                    "cancel", "hold", "cancel_worker"]
    assert record.hold_command_seq > stalled.command_seq
    assert h.adapter.calls == ["cancel", "hold"]
    assert h.canceller.calls == [(0, reason)]
    assert h.lc.outstanding == () and not h.gate._enabled
    # The late, otherwise unexpired result from epoch 0 is rejected without a target.
    h.advance(20)
    late = h.submit(h.result(stalled))
    assert late.disposition == "discarded" and late.proposal.reason == "old_epoch"
    assert h.adapter.calls == ["cancel", "hold"] and h.adapter.targets == []
    assert h.lc.state is S.PAUSED and h.lc.control_epoch == 1


def test_release_fixture_epoch_increments_once_and_resume_needs_fresh_neutral_engagement(h):
    h.active()
    h.lc.dispatch_cycle()
    h.advance(40)  # offline-reference-v1 release fixture: t = 10.04 s
    h.feed()
    h.neutral()
    assert [r.epoch_after for r in h.withdrawals()] == [1]
    h.push()  # nonzero without a newer neutral observation
    assert h.lc.state is S.PAUSED
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.READY and h.lc.control_epoch == 1
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE and h.lc.control_epoch == 1


# ------------------------------------------------------------------- AC2, C06
def test_fresh_feedback_holds_at_current_measured_state(h):
    h.active()
    dispatch = h.lc.dispatch_cycle()
    assert h.submit(h.result(dispatch)).disposition == "accepted_move"
    h.advance(20)
    latest = h.feed((.401, 0., .3))
    h.neutral()
    assert h.adapter.calls == ["apply", "cancel", "hold"]
    held = h.adapter.held[-1]
    assert held is latest.message
    assert held.payload.tcp_pose.position_xyz == (.401, 0., .3)  # measured, not the .405 target


@pytest.mark.parametrize("feedback", ["stale", "invalid"])
def test_stale_or_invalid_feedback_selects_stop(h, feedback):
    h.active()
    if feedback == "stale":
        h.advance(510)
    else:
        h.feed(consistent=False)
    h.neutral()
    record = h.withdrawals()[-1]
    assert_withdrawal(record, before=S.ACTIVE, after=S.PAUSED, epoch=0, action="stop")
    assert "hold" not in record.actions and h.adapter.calls == ["cancel", "stop"]
    assert h.adapter.held == []  # no pose was fabricated from an old target


# ----------------------------------------------------------- AC3, P10, C05
def test_source_age_equality_is_fresh_and_stale_state_withdraws_on_input_path(h):
    h.active()
    sampled = h.coordinator._latest["RobotState"].sampled_ns
    h.clock.now = sampled + 500 * MS
    h.push()
    assert h.lc.state is S.ACTIVE  # age equal to robot_state_timeout_s is allowed
    h.advance(1e-6)  # one nanosecond later
    h.push()
    record = h.withdrawals()[-1]
    assert record.reason == "stale_state" and record.event == "stale_input"
    assert_withdrawal(record, before=S.ACTIVE, after=S.PAUSED, epoch=0, action="stop")
    assert h.lc.pause_causes == {"input"}


@pytest.mark.parametrize("path", ["input", "tick"])
def test_stale_joystick_withdraws_and_holds_at_fresh_feedback(h, path):
    h.active()
    h.advance(510)
    if path == "input":
        h.feed()  # newly received fresh state exposes the stale joystick at once
    else:
        h.lc.tick()  # no new input at all: the watchdog alone withdraws
    record = h.withdrawals()[-1]
    assert record.reason == "stale_joystick"
    action = "hold" if path == "input" else "stop"  # feedback is stale too on the tick path
    assert_withdrawal(record, before=S.ACTIVE, after=S.PAUSED, epoch=0, action=action)


def test_lost_upstream_progress_liveness_watchdog_and_discards_do_not_refresh(h):
    h.active()
    first = h.lc.dispatch_cycle()
    h.advance(50)
    assert h.submit(h.result(first)).disposition == "accepted_move"
    accepted_at = h.clock.now
    assert h.lc.liveness_deadline_ns == accepted_at + 200 * MS
    # Workstation interrupted: duplicates, forged and old messages arrive; inputs stay fresh.
    h.advance(50)
    assert h.submit(h.result(first)).disposition == "discarded"
    forged = h.result(first, tag="forged")
    forged[0]["payload"]["command_seq"] += 5
    assert h.submit(forged).disposition == "discarded"
    while h.clock.now < accepted_at + 200 * MS - 20 * MS:
        h.advance(20)
        h.fresh()
        h.lc.tick()
        assert h.lc.state is S.ACTIVE
    h.clock.now = accepted_at + 200 * MS - 1
    h.lc.tick()
    assert h.lc.state is S.ACTIVE
    h.clock.now = accepted_at + 200 * MS
    h.lc.tick()
    record = h.withdrawals()[-1]
    assert record.reason == "result_liveness_expired" and h.lc.pause_causes == {"computation"}
    assert_withdrawal(record, before=S.ACTIVE, after=S.PAUSED, epoch=0, action="hold")
    assert h.adapter.calls == ["apply", "cancel", "hold"]  # the old target is cancelled


def test_same_tick_acceptance_refreshes_liveness_before_watchdog(h):
    h.active()
    h.advance(100)
    h.fresh()
    dispatch = h.lc.dispatch_cycle()
    h.advance(100)  # exactly engagement + 200 ms
    h.fresh()
    assert h.submit(h.result(dispatch)).disposition == "accepted_move"
    h.lc.tick()
    assert h.lc.state is S.ACTIVE


def test_missed_result_deadline_withdraws_before_liveness(h):
    h.active()
    first = h.lc.dispatch_cycle()
    h.advance(10)
    h.fresh()
    second = h.lc.dispatch_cycle()
    h.advance(40)
    assert h.submit(h.result(first)).disposition == "accepted_move"
    assert h.lc.outstanding == (second,)
    h.clock.now = second.expires_ns - 1
    h.fresh()
    h.lc.tick()
    assert h.lc.state is S.ACTIVE and h.lc.liveness_deadline_ns > second.expires_ns
    h.clock.now = second.expires_ns
    h.fresh()
    h.lc.tick()
    record = h.withdrawals()[-1]
    assert record.reason == "missed_result_deadline" and h.lc.pause_causes == {"computation"}


# ------------------------------------------------------------------------- AC4
def _ev_release(h): h.neutral()
def _ev_disable(h): h.push(enabled=False)
def _ev_stale_state(h): h.advance(510); h.push()
def _ev_liveness(h): h.advance(200); h.fresh(); h.lc.tick()
def _ev_missed(h): h.lc.dispatch_cycle(); h.advance(200); h.fresh(); h.lc.tick()
def _ev_worker_hold(h): h.submit(h.result(h.lc.dispatch_cycle(), hold=True))
def _ev_host_rejection(h): h.submit(h.result(h.lc.dispatch_cycle(), position=(.395, 0., .3)))
def _ev_invalidation(h): h.lc.invalidate_context("scene moved")
def _ev_computation(h): h.lc.computation_status(ready=False, reason="solver worker lost")
def _ev_controller_not_ready(h): h.lc.controller_status(ready=False)
def _ev_controller_fault(h): h.lc.controller_status(ready=False, fault="drive fault")
def _ev_reinitialize(h): h.lc.begin_initialization(h.requests[h.session])
def _ev_restart(h): h.lc.restart_session("operator restart", new_session_id="session-offline-v2")


def _ev_apply_failure(h):
    h.adapter.fail.add("apply")
    h.submit(h.result(h.lc.dispatch_cycle()))


WITHDRAWALS = {
    "release": (_ev_release, S.PAUSED, {"release"}, "hold"),
    "disable": (_ev_disable, S.PAUSED, {"release"}, "hold"),
    "stale_state": (_ev_stale_state, S.PAUSED, {"input"}, "stop"),
    "liveness": (_ev_liveness, S.PAUSED, {"computation"}, "hold"),
    "missed_deadline": (_ev_missed, S.PAUSED, {"computation"}, "hold"),
    "worker_hold": (_ev_worker_hold, S.PAUSED, {"computation"}, "hold"),
    "host_rejection": (_ev_host_rejection, S.PAUSED, {"host_rejection"}, "hold"),
    "invalidation": (_ev_invalidation, S.PAUSED, {"context"}, "hold"),
    "computation": (_ev_computation, S.PAUSED, {"computation"}, "hold"),
    "controller_not_ready": (_ev_controller_not_ready, S.PAUSED, {"controller"}, "hold"),
    "controller_fault": (_ev_controller_fault, S.FAULT, set(), "stop"),
    "apply_failure": (_ev_apply_failure, S.FAULT, set(), "stop"),
    "reinitialize": (_ev_reinitialize, S.PAUSED, {"context"}, "hold"),
    "restart": (_ev_restart, S.UNINITIALIZED, set(), "hold"),
}


@pytest.mark.parametrize("name", WITHDRAWALS)
def test_every_authority_withdrawal_follows_its_transition(h, name):
    event, expected, causes, action = WITHDRAWALS[name]
    h.active()
    old_gate = h.gate
    event(h)
    withdrawals = h.withdrawals()
    assert len(withdrawals) == 1
    assert_withdrawal(withdrawals[0], before=S.ACTIVE, after=expected, epoch=0, action=action)
    assert h.canceller.calls[0][0] == 0
    assert not old_gate._enabled and old_gate._authority_epoch == 1
    assert h.adapter.calls[-2:] == ["cancel", action]
    if name == "reinitialize":
        assert h.lc.state is S.VALIDATING
    elif name == "restart":
        assert h.lc.state is S.UNINITIALIZED and h.session == "session-offline-v2"
    else:
        assert h.lc.state is expected and h.lc.pause_causes == causes
    # No re-entry to ACTIVE without the event-specific recovery and a new engagement.
    h.lc.computation_status(ready=True, reason="restored")
    h.lc.controller_status(ready=True)
    h.fresh()
    h.lc.tick()
    assert h.lc.state is not S.ACTIVE


# ------------------------------------------------------------------------- AC5
def test_recoverable_input_failure_needs_restored_inputs_neutral_and_engagement(h):
    h.active()
    h.advance(510)
    h.push()  # state stale while nonzero input is held
    assert h.lc.state is S.PAUSED
    h.feed()  # restored feedback while input remains nonzero
    h.push()
    h.lc.tick()
    assert h.lc.state is S.PAUSED
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.READY
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE
    assert [t for t in h.transitions() if t[1] is S.ACTIVE][-1][2] == "engagement"


def test_computation_failure_needs_supervisor_readiness_neutral_and_engagement(h):
    h.active()
    h.submit(h.result(h.lc.dispatch_cycle(), hold=True))
    assert h.lc.state is S.PAUSED and h.lc.pause_causes == {"computation"}
    h.lc.computation_status(ready=False, reason="supervisor re-preparing worker")
    h.lc.computation_status(ready=True, reason="worker re-prepared")  # nonzero still held
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.PAUSED
    h.lc.computation_status(ready=False, reason="worker lost again")
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.PAUSED  # neutral alone is insufficient after computation failure
    h.lc.computation_status(ready=True, reason="worker re-prepared with motion disabled")
    assert h.lc.state is S.READY
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE


def _cf_invalid_result(h, mp):
    pair = h.result(h.lc.dispatch_cycle())
    pair[0]["payload"]["horizon_s"] = .1  # snapshot dt_s is 0.2 s
    h.submit(pair)


def _cf_dispatch_failure(h, mp):
    def broken():
        raise RuntimeError("snapshot assembly crashed")

    mp.setattr(h.coordinator, "assemble", broken)
    with pytest.raises(RuntimeError, match="snapshot assembly crashed"):
        h.lc.dispatch_cycle()


COMPUTATION_FAILURES = {
    "liveness": lambda h, mp: _ev_liveness(h),
    "missed_deadline": lambda h, mp: _ev_missed(h),
    "worker_hold": lambda h, mp: _ev_worker_hold(h),
    "invalid_result": _cf_invalid_result,
    "dispatch_failure": _cf_dispatch_failure,
}


@pytest.mark.parametrize("name", COMPUTATION_FAILURES)
def test_computation_failure_needs_readiness_reported_after_the_failure(h, monkeypatch, name):
    """Owner decision 1 without any ready=False signal: readiness from before the
    failure never satisfies recovery or engagement."""
    h.active()
    assert h.lc.computation_ready  # the supervisor reported ready before engagement
    COMPUTATION_FAILURES[name](h, monkeypatch)
    monkeypatch.undo()
    assert h.lc.state is S.PAUSED and h.lc.pause_causes == {"computation"}
    assert not h.lc.computation_ready
    for _ in range(3):  # fresh inputs, a newer neutral and eligible feedback are not enough
        h.advance(20)
        h.fresh((0., 0.))
        h.lc.tick()
        assert h.lc.state is S.PAUSED
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.PAUSED
    h.lc.computation_status(ready=True, reason="worker re-prepared with motion disabled")
    assert h.lc.state is S.PAUSED  # a held nonzero input is not a newer neutral
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.READY
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE


def test_engagement_requires_computation_controller_and_eligible_feedback(h):
    h.feed()
    h.neutral()
    h.lc.initialize(h.requests[h.session])
    h.push()
    assert h.lc.state is S.READY
    refused = [r for r in h.lc.records if r.event == "engagement_refused"][-1]
    assert refused.reason == "controller_not_ready,computation_not_ready"
    h.lc.controller_status(ready=True)
    h.lc.computation_status(ready=True, reason="prepared")
    h.push()  # still unarmed: a refused attempt needs neutral again
    assert h.lc.state is S.READY
    h.neutral()
    h.feed(consistent=False)
    h.push()
    assert h.lc.state is S.READY
    assert [r for r in h.lc.records if r.event == "engagement_refused"][-1].reason == "ineligible_feedback"
    h.feed()
    h.neutral()
    h.push()
    assert h.lc.state is S.ACTIVE


# ------------------------------------------------------------------------- AC6
def test_ordinary_nonzero_updates_keep_epoch_and_neutral_repeats_add_no_authority(h):
    h.active()
    dispatch = h.lc.dispatch_cycle()
    for u in ((.8, .2), (.9, .1), (.9, -.1)):
        h.advance(20)
        h.fresh(u)
    assert h.lc.control_epoch == 0 and h.lc.state is S.ACTIVE
    decision = h.submit(h.result(dispatch))
    assert decision.disposition == "accepted_move"  # pinned seq is older than the live seq
    assert dispatch.inputs.joystick.message.meta.sequence < h.coordinator._latest["JoystickSample"].message.meta.sequence
    h.advance(20)
    h.fresh((0., 0.))
    calls, cancels = list(h.adapter.calls), list(h.canceller.calls)
    for _ in range(5):
        h.advance(20)
        h.fresh((0., 0.))
        h.lc.tick()
    assert h.lc.state is S.READY and h.lc.control_epoch == 1
    assert h.adapter.calls == calls and h.canceller.calls == cancels
    assert not h.gate._enabled and len(h.withdrawals()) == 1


# ------------------------------------------------------------------- AC7, C07
def test_ordinary_release_reuses_the_valid_bundle(h):
    bundle = h.active()
    h.neutral()
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.READY and h.store.acquire() is bundle
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE and h.store.acquire() is bundle
    assert [r.event for r in h.lc.records].count("activation") == 1
    assert h.lc.dispatch_cycle().inputs.bundle is bundle


def test_ready_engagement_with_stale_feedback_is_refused(h):
    h.active()
    h.neutral()
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.READY
    h.advance(490)
    h.push()  # state sampled 490 ms ago + no newer feedback -> still within 0.5 s
    assert h.lc.state is S.ACTIVE
    h.neutral()
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.READY
    h.advance(510)
    h.push()
    assert h.lc.state is S.READY
    assert [r for r in h.lc.records if r.event == "engagement_refused"][-1].reason == "stale_state"


def test_context_invalidation_requires_reinitialization_and_new_engagement(h):
    old_bundle = h.active()
    h.lc.invalidate_context("scene moved")
    assert h.lc.state is S.PAUSED and h.store.acquire() is None
    record = h.withdrawals()[-1]
    assert_withdrawal(record, before=S.ACTIVE, after=S.PAUSED, epoch=0, action="hold")
    for u in ((0., 0.), (1., 0.), (0., 0.)):
        h.advance(20)
        h.fresh(u)
        h.lc.tick()
        assert h.lc.state is S.PAUSED
    h.push()  # operator holds input while the context is re-validated
    outcome = h.lc.initialize(h.requests[h.session])
    assert h.lc.state is S.READY and outcome.bundle is not old_bundle
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.READY
    h.advance(20)
    h.fresh((0., 0.))
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE and h.lc.dispatch_cycle().inputs.bundle is outcome.bundle


def test_rejected_validation_pauses_until_a_valid_context_is_activated(h):
    h.feed()
    h.neutral()
    bad = replace(h.requests[h.session], observation_id="observation-not-used-by-geometry")
    outcome = h.lc.initialize(bad)
    assert outcome.status == "rejected" and h.lc.state is S.PAUSED
    h.advance(20)
    h.fresh((0., 0.))
    assert h.lc.state is S.PAUSED
    assert h.lc.initialize(h.requests[h.session]).status == "activated"
    assert h.lc.state is S.READY


# ------------------------------------------------------------------------- AC8
def test_activation_and_async_signals_never_resume_held_input(h):
    h.feed()
    h.push()  # nonzero held before initialization finishes
    h.lc.initialize(h.requests[h.session])
    assert h.lc.state is S.READY
    h.lc.controller_status(ready=True)
    h.lc.computation_status(ready=True, reason="prepared")
    for _ in range(3):
        h.advance(20)
        h.fresh()
        h.lc.tick()
    assert h.lc.state is S.READY and not h.gate._enabled
    h.advance(20)
    h.fresh((0., 0.))
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE


def test_superseded_or_late_initialization_never_activates(h):
    h.feed()
    h.neutral()
    first = h.lc.begin_initialization(h.requests[h.session])
    early = h.lc.prepare_initialization(first)
    assert early.status == "prepared"
    second = h.lc.begin_initialization(h.requests[h.session])
    assert h.lc.prepare_initialization(first).status == "superseded"
    assert h.lc.complete_initialization(early).status == "superseded"
    assert h.lc.state is S.VALIDATING and h.store.acquire() is None
    assert h.lc.complete_initialization(h.lc.prepare_initialization(second)).status == "activated"
    assert h.lc.state is S.READY


def test_late_prior_authority_moves_and_holds_do_not_disturb_newer_decisions(h):
    h.active()
    old_move = h.lc.dispatch_cycle()
    old_hold = h.lc.dispatch_cycle()
    h.advance(20)
    h.fresh((0., 0.))
    h.advance(20)
    h.fresh((0., 0.))
    h.advance(20)
    h.fresh()
    assert h.lc.state is S.ACTIVE and h.lc.control_epoch == 1
    current = h.lc.dispatch_cycle()
    assert h.submit(h.result(current)).disposition == "accepted_move"
    calls, previous = list(h.adapter.calls), h.coordinator._previous
    for pair in (h.result(old_move), h.result(old_hold, hold=True)):
        decision = h.submit(pair)
        assert decision.disposition == "discarded" and decision.proposal.reason == "old_epoch"
    assert h.adapter.calls == calls and h.coordinator._previous is previous
    assert h.lc.state is S.ACTIVE and h.gate._enabled


# ------------------------------------------------------------------------- AC9
def _fault_controller(h): h.lc.controller_status(ready=False, fault="drive fault")
def _fault_failed_stop(h): h.adapter.fail.add("stop"); h.advance(510); h.neutral()
def _fault_failed_hold(h): h.adapter.fail.add("hold"); h.neutral()
def _fault_apply(h): h.adapter.fail.add("apply"); h.submit(h.result(h.lc.dispatch_cycle()))


@pytest.mark.parametrize("cause", [_fault_controller, _fault_failed_stop, _fault_failed_hold, _fault_apply],
                         ids=["controller_fault", "failed_stop", "failed_hold_falls_back", "apply_failure"])
def test_fault_latches_until_clearance_and_explicit_rearm(h, cause):
    h.active()
    cause(h)
    assert h.lc.state is S.FAULT and not h.gate._enabled
    assert h.adapter.calls[-1] == "stop"
    h.adapter.fail.clear()
    for u in ((0., 0.), (1., 0.)):
        h.advance(20)
        h.fresh(u)
        h.lc.tick()
        assert h.lc.state is S.FAULT
    with pytest.raises(LifecycleError, match="fault_latched"):
        h.lc.begin_initialization(h.requests[h.session])
    with pytest.raises(LifecycleError, match="fault_not_cleared"):
        h.lc.rearm("operator rearm")
    h.lc.computation_status(ready=True, reason="unrelated signal")
    assert h.lc.state is S.FAULT and not h.lc.fault_cleared
    h.lc.controller_status(ready=True)
    assert h.lc.fault_cleared and h.lc.state is S.FAULT
    h.lc.rearm("operator rearm after clearance")
    assert h.lc.state is S.UNINITIALIZED and h.store.acquire() is None
    h.advance(20)
    h.fresh((0., 0.))
    h.push()
    assert h.lc.state is S.UNINITIALIZED
    h.neutral()
    h.lc.initialize(h.requests[h.session])
    assert h.lc.state is S.READY
    h.push()
    assert h.lc.state is S.ACTIVE


def test_fault_reported_again_revokes_clearance(h):
    h.active()
    h.lc.controller_status(ready=False, fault="drive fault")
    h.lc.controller_status(ready=True)
    h.lc.controller_status(ready=False, fault="drive fault again")
    with pytest.raises(LifecycleError, match="fault_not_cleared"):
        h.lc.rearm("operator rearm")
    with pytest.raises(LifecycleError, match="not_faulted"):
        Harness(h.build).lc.rearm("nothing latched")


def test_unrecoverable_host_condition_during_validation_latches_fault(h, monkeypatch):
    h.feed()
    h.neutral()
    attempt = h.lc.begin_initialization(h.requests[h.session])

    def broken(_):
        raise RuntimeError("validator host crashed")

    monkeypatch.setattr(h.store, "prepare", broken)
    with pytest.raises(RuntimeError, match="validator host crashed"):
        h.lc.prepare_initialization(attempt)
    record = h.withdrawals()[-1]
    assert (record.state_before, record.state_after) == (S.VALIDATING, S.FAULT)
    assert h.adapter.calls == ["cancel", "stop"]


# ------------------------------------------------------------------ AC10, P09
def test_session_restart_withdraws_before_clearing_and_old_session_cannot_revive(h):
    h.active()
    old_runtime = h.lc.runtime
    pending = h.lc.dispatch_cycle()
    stale_pair = h.result(pending)
    new_session = h.lc.restart_session("operator restart", new_session_id="session-offline-v2")
    withdrawal, session = h.lc.records[-2:]
    assert withdrawal.kind == "withdrawal" and session.kind == "session"
    assert_withdrawal(withdrawal, before=S.ACTIVE, after=S.UNINITIALIZED, epoch=0, action="hold")
    assert withdrawal.session_id == SESSION and session.session_id == new_session == "session-offline-v2"
    assert session.detail == f"previous_session={SESSION}" and session.epoch_after == 0
    assert h.lc.retired_sessions == (old_runtime,) and old_runtime.store.acquire() is None
    assert not old_runtime.gate._enabled and h.lc.state is S.UNINITIALIZED
    assert h.canceller.calls == [(0, "operator restart")]
    # Old results and inputs are rejected regardless of sequence or timestamp.
    assert h.submit(stale_pair).disposition == "discarded"
    assert old_runtime.gate.submit(stale_pair[0], weights=stale_pair[1]).disposition == "discarded"
    assert h.lc.on_input(h.joystick(session=SESSION)) is None
    assert h.lc.records[-1].reason == "session_mismatch"
    with pytest.raises(LifecycleError, match="session_reused"):
        h.lc.restart_session("again", new_session_id=SESSION)
    assert h.lc.state is S.UNINITIALIZED  # refused before any withdrawal
    # Fresh session counters: snapshot and command sequences restart at one.
    h.ready()
    h.push()
    fresh = h.lc.dispatch_cycle()
    assert (fresh.inputs.snapshot.snapshot_seq, fresh.command_seq, fresh.inputs.snapshot.control_epoch) == (1, 1, 0)


def test_replayed_older_source_sequence_changes_nothing(h):
    h.active()
    h.advance(20)
    h.fresh()
    replay = h.joystick((0., 0.), sequence=1)  # older neutral sample replayed
    assert h.lc.on_input(replay) is None
    assert h.lc.state is S.ACTIVE and h.lc.records[-1].reason == "replayed_input"
    deadline = h.lc.liveness_deadline_ns
    h.lc.on_input(h.state(sequence=1))
    assert h.lc.liveness_deadline_ns == deadline


def test_fault_latch_survives_session_restart(h):
    h.active()
    h.lc.controller_status(ready=False, fault="drive fault")
    h.lc.restart_session("operator restart", new_session_id="session-offline-v2")
    assert h.lc.state is S.FAULT and h.session == "session-offline-v2"
    assert h.adapter.calls[-2:] == ["cancel", "stop"]
    with pytest.raises(LifecycleError, match="fault_not_cleared"):
        h.lc.rearm("operator rearm")
    h.lc.controller_status(ready=True)
    h.lc.rearm("operator rearm after clearance")
    assert h.lc.state is S.UNINITIALIZED


def test_fault_latch_survives_clock_domain_reset(h):
    h.active()
    h.lc.controller_status(ready=False, fault="drive fault")
    h.clock_id = "synthetic-monotonic-epoch-2"
    session = h.lc.clock_domain_reset("adapter reported clock epoch change")
    assert session != SESSION and h.lc.state is S.FAULT
    assert h.adapter.calls[-2:] == ["cancel", "stop"]
    with pytest.raises(LifecycleError, match="fault_latched"):
        h.lc.begin_initialization(h.requests[h.session])
    with pytest.raises(LifecycleError, match="fault_not_cleared"):
        h.lc.rearm("operator rearm")
    h.lc.controller_status(ready=True)
    h.lc.rearm("operator rearm after clearance")
    assert h.lc.state is S.UNINITIALIZED


def _restart(h):
    h.lc.restart_session("operator restart", new_session_id="session-offline-v2")


def _clock_reset(h):
    h.clock_id = "synthetic-monotonic-epoch-2"
    h.lc.clock_domain_reset("adapter reported clock epoch change")


@pytest.mark.parametrize("new_session", [_restart, _clock_reset], ids=["restart", "clock_reset"])
def test_failed_stop_while_faulted_revokes_earlier_clearance(h, new_session):
    h.active()
    h.lc.controller_status(ready=False, fault="drive fault")
    h.lc.controller_status(ready=True)
    assert h.lc.fault_cleared
    h.adapter.fail.add("stop")
    new_session(h)
    assert h.lc.state is S.FAULT and not h.lc.fault_cleared
    with pytest.raises(LifecycleError, match="fault_not_cleared"):
        h.lc.rearm("operator rearm")
    h.adapter.fail.clear()
    h.lc.controller_status(ready=True)  # healthy report after the failed stop
    h.lc.rearm("operator rearm after clearance")
    assert h.lc.state is S.UNINITIALIZED


@pytest.mark.parametrize("new_session", [_restart, _clock_reset], ids=["restart", "clock_reset"])
def test_new_session_needs_new_computation_readiness(h, new_session):
    h.active()
    new_session(h)
    assert h.lc.state is S.UNINITIALIZED and not h.lc.computation_ready
    h.feed()
    h.neutral()
    assert h.lc.initialize(h.requests[h.session]).status == "activated"
    h.push()
    assert h.lc.state is S.READY
    assert [r for r in h.lc.records if r.event == "engagement_refused"][-1].reason == "computation_not_ready"
    h.lc.computation_status(ready=True, reason="solver prepared for the new session")
    h.neutral()
    h.push()
    assert h.lc.state is S.ACTIVE


@pytest.mark.parametrize("stop_fails", [False, True])
def test_clock_domain_failure_stops_directly_and_starts_new_session(h, stop_fails):
    h.active()
    old_runtime = h.lc.runtime
    backwards = h.joystick(at=h.clock.now - 2 * NS)
    h.clock.now -= NS  # local monotonic clock moved backwards
    h.clock_id = "synthetic-monotonic-epoch-2"  # the host's new clock domain
    if stop_fails:
        h.adapter.fail.add("stop")
    assert h.lc.on_input(backwards) is None
    withdrawal, session = h.lc.records[-2:]
    assert withdrawal.event == "clock_failure" and withdrawal.at_ns is None
    assert list(withdrawal.actions) == ["withdraw_authority", "increment_epoch", "cancel", "stop", "cancel_worker"]
    assert h.adapter.held == [] and not old_runtime.gate._enabled
    assert session.kind == "session" and h.session != SESSION
    assert h.lc.state is (S.FAULT if stop_fails else S.UNINITIALIZED)
    assert h.lc.on_input(h.joystick(session=SESSION)) is None


def test_explicit_clock_domain_reset_starts_new_session(h):
    h.active()
    h.clock_id = "synthetic-monotonic-epoch-2"
    session = h.lc.clock_domain_reset("adapter reported clock epoch change")
    assert session != SESSION and h.lc.state is S.UNINITIALIZED
    assert h.adapter.calls == ["cancel", "stop"]


def test_reentrant_adapter_call_is_refused_and_latches_fault(h):
    """An adapter violating the protocol by calling back during a withdrawal."""
    h.active()
    recording_hold = h.adapter.hold

    def reentrant_hold(measured_state):
        recording_hold(measured_state)
        h.lc.controller_status(ready=False, fault="reported from inside hold")

    h.adapter.hold = reentrant_hold
    h.neutral()
    record = h.withdrawals()[-1]
    assert (record.event, record.state_before, record.state_after) == ("release", S.ACTIVE, S.FAULT)
    assert "reentrant_call" in record.adapter_error
    assert h.lc.state is S.FAULT and h.adapter.calls[-2:] == ["hold", "stop"]
    assert not [r for r in h.lc.records if r.event == "controller_fault"]
    assert len(h.withdrawals()) == 1


def test_reentrant_canceller_call_is_refused_and_recorded(h):
    h.active()

    class Reentrant:
        def cancel(self, control_epoch, reason):
            h.lc.restart_session("canceller misbehaves")

    h.lc._canceller = Reentrant()
    h.neutral()
    record = h.withdrawals()[-1]
    assert h.lc.state is S.PAUSED and "reentrant_call" in record.detail
    assert h.session == SESSION and len(h.withdrawals()) == 1


def test_worker_cancel_failure_is_recorded_not_awaited(h):
    h.active()
    h.canceller.fail = True
    h.neutral()
    record = h.withdrawals()[-1]
    assert h.lc.state is S.PAUSED and "worker unreachable" in record.detail


# ------------------------------------------------------------- retained trace
def test_retained_lifecycle_trace(h, tmp_path):
    scenarios = {}

    def mark(name):
        scenarios[name] = len(h.lc.records)

    mark("initialize_and_engage")
    bundle = h.active()
    accepted = h.lc.dispatch_cycle()
    h.advance(50)
    h.submit(h.result(accepted))
    mark("C02_release_during_stalled_solve")
    stalled = h.lc.dispatch_cycle()
    h.advance(20)
    h.fresh((0., 0.))
    late = h.submit(h.result(stalled))
    mark("C07_release_reuses_bundle")
    h.advance(20)
    h.fresh((0., 0.))
    h.advance(20)
    h.fresh()
    mark("C05_liveness_watchdog")
    h.advance(200)
    h.fresh()
    h.lc.tick()
    mark("P10_C06_stale_state_stop")
    h.advance(20)
    h.lc.computation_status(ready=True, reason="worker re-prepared")
    h.fresh((0., 0.))
    h.advance(20)
    h.fresh()
    h.advance(510)
    h.push()
    mark("C07_invalidation_requires_reinitialization")
    h.advance(20)
    h.fresh((0., 0.))
    h.lc.invalidate_context("scene moved")
    h.lc.initialize(h.requests[h.session])
    h.advance(20)
    h.fresh()
    mark("fault_and_rearm")
    h.lc.controller_status(ready=False, fault="synthetic drive fault")
    h.lc.controller_status(ready=True)
    h.lc.rearm("operator rearm after clearance")
    mark("P09_session_restart")
    h.lc.restart_session("operator restart", new_session_id="session-offline-v2")
    stale = h.lc.on_input(h.joystick(session=SESSION))
    trace = {
        "profile": "offline-reference-v1", "configuration": to_dict(bundle.resolved.configuration),
        "liveness_timeout_s": 0.2, "evidence": "software simulation, controllable clock, recording adapter",
        "scenarios": scenarios, "records": [r.to_dict() for r in h.lc.records],
        "adapter_calls": h.adapter.calls, "worker_cancellations": h.canceller.calls,
        "late_prior_epoch_result": {"disposition": late.disposition, "reason": late.proposal.reason},
        "old_session_input_accepted": stale is not None,
    }
    (tmp_path / "lifecycle-trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    path = [(r["state_before"], r["state_after"]) for r in trace["records"] if r["state_before"] != r["state_after"]]
    assert path == [
        ("UNINITIALIZED", "VALIDATING"), ("VALIDATING", "READY"), ("READY", "ACTIVE"),
        ("ACTIVE", "PAUSED"),                                       # C02 release
        ("PAUSED", "READY"), ("READY", "ACTIVE"),                   # C07 same bundle
        ("ACTIVE", "PAUSED"),                                       # C05 liveness
        ("PAUSED", "READY"), ("READY", "ACTIVE"), ("ACTIVE", "PAUSED"),  # P10/C06 stale state
        ("PAUSED", "READY"), ("READY", "PAUSED"),                   # C07 invalidation
        ("PAUSED", "VALIDATING"), ("VALIDATING", "READY"), ("READY", "ACTIVE"),
        ("ACTIVE", "FAULT"), ("FAULT", "UNINITIALIZED"),            # fault and rearm
    ]
    stops = [r["reason"] for r in trace["records"] if "stop" in r["actions"]]
    # Restart after rearm has no context to derive a hold from, so it stops.
    assert stops == ["stale_state", "synthetic drive fault", "operator restart"]
    assert trace["late_prior_epoch_result"] == {"disposition": "discarded", "reason": "old_epoch"}
    assert not trace["old_session_input_accepted"]
