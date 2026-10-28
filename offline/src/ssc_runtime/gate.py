"""Local command acceptance with an explicit, synchronous adapter boundary.

This module consumes lifecycle authority. It does not infer engagement, run a
watchdog, advance a plant, or claim a submitted command was physically executed.
Engagement, watchdogs and recovery live in lifecycle.LifecycleCoordinator.
"""

from dataclasses import dataclass
import math
from typing import Protocol
from uuid import uuid4

from scipy.spatial.transform import Rotation

from ssc_core import (CommandAcceptanceRecord, Message, NUMERIC_CANDIDATES_V2,
                      TimePoint, estimate_intent, get_intent_config, get_solver_config, parse_message, to_dict)

from .constraints import SyntheticHostChecker, basic_step_constraints, measured_view
from .snapshots import AcceptedCommand, NS, ResultArrival, SnapshotCoordinator, SnapshotError, _duration


@dataclass(frozen=True, slots=True)
class AcceptedTarget:
    """Request to install a bounded target; not measured feedback."""

    result: Message
    accepted_ns: int
    expires_ns: int
    u_xy: tuple
    measured_state: Message


class CommandAdapter(Protocol):
    """Trusted nonblocking local submissions. Raise on a failed submission.

    No runtime reentry, worker/network waits, or inference inside these calls.
    Successful return confirms submission only, never physical execution.
    """

    def apply(self, target: AcceptedTarget) -> None: ...
    def cancel(self) -> None: ...
    def hold(self, measured_state: Message) -> None: ...
    def stop(self) -> None: ...


@dataclass(frozen=True, slots=True)
class GateDecision:
    proposal: CommandAcceptanceRecord
    disposition: str
    actions: tuple[str, ...] = ()
    applied: AcceptedCommand | None = None
    lifecycle_request: str | None = None
    adapter_error: str | None = None
    observed_execution: None = None


class CommandGate:
    """One trusted gate per SnapshotCoordinator; worker input is only submit().

    Lifecycle alone calls set_authority()/local_hold()/local_stop(). Initialization
    is disabled. A current failure interlocks motion until lifecycle explicitly
    grants permission again; LifecycleCoordinator owns recovery/epoch transitions.
    """

    def __init__(self, coordinator: SnapshotCoordinator, adapter: CommandAdapter, *,
                 estimator_config_id: str = "intent-candidates-v2"):
        if type(coordinator) is not SnapshotCoordinator:
            raise TypeError("expected SnapshotCoordinator")
        if not estimator_config_id or type(estimator_config_id) is not str:
            raise ValueError("expected host estimator identity")
        for method in ("apply", "cancel", "hold", "stop"):
            if not callable(getattr(adapter, method, None)):
                raise TypeError(f"adapter is missing {method}")
        self._coordinator, self._adapter = coordinator, adapter
        self._estimator = estimator_config_id
        self._checker = SyntheticHostChecker()
        self._enabled = self._controller_ready = False
        self._authority_epoch = coordinator.control_epoch
        self._last_sequence = self._last_move_snapshot = 0
        self._accepted_results: set[str] = set()
        self._last_eligible_result_ns: int | None = None
        self._decisions: list[GateDecision] = []
        self._arrivals: list[ResultArrival] = []

    @property
    def decisions(self):
        with self._coordinator._lock:
            return tuple(self._decisions)

    @property
    def last_eligible_result_ns(self):
        with self._coordinator._lock:
            return self._last_eligible_result_ns

    @property
    def arrivals(self):
        """First-class local arrival records; remote produced_at is never used."""
        with self._coordinator._lock:
            return tuple(self._arrivals)

    def set_authority(self, *, control_epoch: int, motion_enabled: bool, controller_ready: bool):
        """Trusted lifecycle input. Never expose this method to worker messages.

        Revocation must be accompanied by local_hold() on the local event path.
        The full release/rearm state machine is intentionally separate.
        """
        with self._coordinator._lock:
            if (type(control_epoch) is not int or control_epoch != self._coordinator.control_epoch or
                    type(motion_enabled) is not bool or type(controller_ready) is not bool):
                raise ValueError("invalid lifecycle authority")
            self._authority_epoch = control_epoch
            self._enabled, self._controller_ready = motion_enabled, controller_ready

    def _stamp(self, now):
        return TimePoint(self._coordinator._clock_id, now // NS, now % NS)

    def _record(self, result_id, accepted, reason, now, applied_id=None):
        return CommandAcceptanceRecord(result_id, accepted, reason, self._stamp(now), applied_id)

    def _finish(self, decision):
        self._decisions.append(decision)
        return decision

    def _discard(self, result_id, reason, now):
        return self._finish(GateDecision(self._record(result_id, False, reason, now), "discarded"))

    def _accepted(self, message, reason, now):
        record = self._record(message.payload.result_id, True, reason, now, str(uuid4()))
        entry = self._coordinator._record_acceptance(message, record, register_result=False)
        self._last_sequence = message.payload.command_seq
        self._accepted_results.add(message.payload.result_id)
        if message.payload.status == "move":
            self._last_move_snapshot = entry.source.snapshot.snapshot_seq
        return entry

    def _feedback(self, bundle, now):
        """Holding requires fresh compatible measured feedback, not joystick."""
        if bundle is None:
            return None
        state = self._coordinator._latest.get("RobotState")
        if state is None:
            return None
        try:
            self._coordinator._fresh(state, bundle.resolved.configuration.robot_state_timeout_s, now)
            value = state.message.payload
            # Zero-length check also verifies state/model agreement and bounds.
            report = self._checker(candidate_pose=value.tcp_pose, measured_state=measured_view(value),
                                   u_xy=(0., 0.), dt_s=bundle.resolved.configuration.control_period_s,
                                   configuration=bundle.resolved.configuration)
            self._coordinator._fresh(state, bundle.resolved.configuration.robot_state_timeout_s,
                                     self._coordinator._now())
            return state.message if report.feasible else None
        except Exception:
            return None

    def _hold_actions(self, bundle, *, force_stop=False, expires_ns=None):
        actions, errors = [], []
        state = None if force_stop else self._feedback(bundle, self._coordinator._now())
        checked = self._coordinator._now()
        if expires_ns is not None and checked >= expires_ns:
            return (), None, False, checked
        failed = False
        try:
            actions.append("cancel")
            self._adapter.cancel()
        except Exception as exc:
            failed = True
            errors.append(f"cancel: {type(exc).__name__}: {exc}")
        if state is not None:
            try:
                sample = self._coordinator._latest["RobotState"]
                self._coordinator._fresh(sample, bundle.resolved.configuration.robot_state_timeout_s,
                                         self._coordinator._now())
            except SnapshotError:
                state = None
        action = "hold" if state is not None and not failed else "stop"
        try:
            actions.append(action)
            if action == "hold":
                self._adapter.hold(state)
            else:
                self._adapter.stop()
        except Exception as exc:
            failed = True
            errors.append(f"{action}: {type(exc).__name__}: {exc}")
            if action == "hold":
                try:
                    actions.append("stop")
                    self._adapter.stop()
                except Exception as stop_exc:
                    errors.append(f"stop: {type(stop_exc).__name__}: {stop_exc}")
        return tuple(actions), "; ".join(errors) or None, failed, checked

    def _local_message(self, reason, now):
        coordinator = self._coordinator
        sequence = coordinator.allocate_local_command_sequence()
        result_id = str(uuid4())
        return parse_message({"message_type": "ControlResult", "meta": {
            "schema_version": "2.0", "session_id": coordinator._store.session_id,
            "message_id": str(uuid4()), "source_id": "local-command-gate",
            "sequence": sequence, "produced_at": to_dict(self._stamp(now)),
        }, "payload": {
            "result_id": result_id, "snapshot_id": None, "selection_id": None,
            "control_epoch": coordinator.control_epoch, "command_seq": sequence,
            "source_joystick_seq": None, "source_robot_state_seq": None,
            "status": "hold", "evaluation_valid": False, "solver_status": "not_run",
            "constraints_satisfied": None, "solve_time_s": 0, "candidate_evaluation": None,
            "reason": reason,
        }}, policy=NUMERIC_CANDIDATES_V2)

    def _local_hold(self, reason, bundle, now, *, fault=False):
        self._enabled = False
        message = self._local_message(reason, now)
        actions, error, failed, checked = self._hold_actions(bundle, force_stop=fault)
        applied = None if failed else self._accepted(message, reason, checked)
        record = applied.acceptance if applied else self._record(message.payload.result_id, False, "adapter_failure", checked)
        return GateDecision(record, "local_hold" if applied else "adapter_failure", actions, applied,
                            "FAULT" if fault or failed else "PAUSED", error)

    def local_hold(self, reason: str) -> GateDecision:
        """Trusted coordinator path; allocates a new sequence, never a fake snapshot."""
        return self._trusted_local(reason, fault=False)

    def local_stop(self, reason: str) -> GateDecision:
        """Trusted fault path: cancel then stop, without needing fresh feedback.

        Always requests FAULT. Used by the lifecycle for controller faults and
        latched conditions; a worker can never reach this method.
        """
        return self._trusted_local(reason, fault=True)

    def _trusted_local(self, reason, *, fault):
        if type(reason) is not str or not reason.strip():
            raise ValueError("hold reason required")
        with self._coordinator.command_transaction() as bundle:
            now = self._coordinator._now()
            return self._finish(self._local_hold(reason, bundle, now, fault=fault))

    def _reject_current(self, result_id, reason, bundle, now, *, fault=False):
        fallback = self._local_hold(reason, bundle, now, fault=fault)
        self._finish(fallback)
        return self._finish(GateDecision(self._record(result_id, False, reason, now), "rejected",
                                        fallback.actions, fallback.applied, fallback.lifecycle_request,
                                        fallback.adapter_error))

    def _identities(self, message, selection_data, dispatch, registry):
        result, source = message.payload, dispatch.inputs
        if selection_data is not None:
            selection_message = self._coordinator._message(selection_data)
            if selection_message.message_type != "CandidateSelection":
                raise ValueError("selection_type")
            registry.register_message(selection_message)
        selected = None
        if result.selection_id is not None:
            resolved = registry.resolve_selection(result.selection_id)
            selection = resolved.selection
            if (selection.snapshot_id != source.snapshot.snapshot_id or
                    selection.estimator_config_id != self._estimator or resolved.snapshot != source.resolved):
                raise ValueError("selection_snapshot_mismatch")
            expected = estimate_intent(source.resolved, get_intent_config(self._estimator),
                selection_id=selection.selection_id, policy=NUMERIC_CANDIDATES_V2).selection
            if selection != expected:
                raise ValueError("selection_recompute_mismatch")
            selected = next((point for point in source.bundle.resolved.candidates
                             if point.candidate_id == selection.selected_candidate_id), None)
        evaluation = result.candidate_evaluation
        if result.status == "move":
            if selected is None or evaluation is None or evaluation.candidate_id != selected.candidate_id:
                raise ValueError("evaluation_candidate_mismatch")
            # Fixed metric is independently recomputed, not trusted worker output.
            distance = math.dist(result.next_tcp_pose.position_xyz, selected.position_xyz)
            solver = get_solver_config("solver-candidates-v2")
            raw_g = distance - solver.distance_tolerance_m
            scale = solver.distance_scale_m
            if evaluation.normalization_scale != scale:
                raise ValueError("evaluation_scale_mismatch")
            for actual, expected in ((evaluation.distance_m, distance), (evaluation.raw_g, raw_g),
                                     (evaluation.penalty, max(0., raw_g / scale) ** 2)):
                if not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9):
                    raise ValueError("evaluation_metric_mismatch")
            if selection.assistance_strength == 0 and not math.isclose(
                    result.next_tcp_pose.position_xyz[2], source.robot_state.message.payload.tcp_pose.position_xyz[2],
                    rel_tol=0., abs_tol=1e-12):
                raise ValueError("unassisted_height_changed")
            if result.constraints_satisfied != (raw_g <= 1e-9):
                raise ValueError("evaluation_satisfaction_mismatch")
        registry.register_message(message)

    def submit(self, data: dict | Message, *, selection: dict | Message | None = None) -> GateDecision:
        """Immediate local result arrival and decision; remote stamps grant nothing.

        Malformed/unattributable envelopes are discarded. Structurally valid,
        current causal failures route through a separate trusted local hold.
        """
        coordinator = self._coordinator
        with coordinator._lock:
            received_ns = coordinator._now()
        identity_error = None
        try:
            message = coordinator._message(data)
            if message.message_type != "ControlResult":
                raise ValueError("result_type")
        except (ValueError, TypeError) as exc:
            return self._malformed(data, str(exc))
        # Registry resolution may wait for preparation, without blocking input,
        # local holds or epoch/context withdrawal. Recheck everything live below.
        try:
            dispatch = coordinator.lookup_dispatch(message.payload.snapshot_id)
            with coordinator._lock:
                self._arrivals.append(ResultArrival(dispatch, message, received_ns,
                    received_ns < dispatch.expires_ns, received_ns < dispatch.solve_deadline_ns))
            with coordinator._store.artifacts() as registry:
                self._identities(message, selection, dispatch, registry)
        except Exception as exc:
            identity_error = str(exc)
        with coordinator.command_transaction() as bundle:
            now = coordinator._now()
            result = message.payload
            try:
                dispatch = coordinator.lookup_dispatch(result.snapshot_id)
            except SnapshotError:
                return self._discard(result.result_id, "unknown_dispatch", now)
            source = dispatch.inputs
            # All supersession checks precede *any* cancel/hold path.
            for bad, reason in (
                (result.control_epoch != coordinator.control_epoch or result.control_epoch != source.snapshot.control_epoch, "old_epoch"),
                (source.bundle is not bundle, "superseded_bundle"),
                (result.command_seq != dispatch.command_seq, "forged_command_sequence"),
                (result.command_seq <= self._last_sequence or result.result_id in self._accepted_results, "superseded_command"),
                (source.snapshot.snapshot_seq <= self._last_move_snapshot, "superseded_snapshot"),
                (now >= source.expires_ns, "snapshot_expired"),
            ):
                if bad:
                    return self._discard(result.result_id, reason, now)
            if not self._enabled or self._authority_epoch != coordinator.control_epoch:
                return self._discard(result.result_id, "motion_authority_disabled", now)
            try:
                if not self._controller_ready:
                    return self._reject_current(result.result_id, "controller_not_ready", bundle, now, fault=True)
                if (result.source_joystick_seq != source.joystick.message.meta.sequence or
                        result.source_robot_state_seq != source.robot_state.message.meta.sequence):
                    raise ValueError("source_sequence_mismatch")
                if result.status == "move" and result.horizon_s != source.snapshot.dt_s:
                    raise ValueError("horizon_mismatch")
                # Local submission has no separate remote completion proof. The
                # solver must deliver within its original local solve budget.
                if received_ns >= dispatch.solve_deadline_ns and result.status == "move":
                    raise ValueError("solve_deadline_expired")
                if result.solve_time_s * NS > dispatch.solve_deadline_ns - dispatch.dispatched_ns:
                    raise ValueError("solve_budget_exceeded")
                if identity_error:
                    raise ValueError(identity_error)
                config = source.bundle.resolved.configuration
                joystick, state = coordinator._inputs(bundle, now)
                if not coordinator._enabled(joystick):
                    raise ValueError("motion_input_disabled")
                if result.status == "hold":
                    actions, error, failed, checked = self._hold_actions(bundle, expires_ns=source.expires_ns)
                    if not actions:
                        return self._discard(result.result_id, "snapshot_expired", checked)
                    self._enabled = False
                    if failed:
                        return self._finish(GateDecision(self._record(result.result_id, False, "adapter_failure", now),
                                                        "adapter_failure", actions, lifecycle_request="FAULT", adapter_error=error))
                    applied = self._accepted(message, "worker_hold: " + result.reason, checked)
                    self._last_eligible_result_ns = checked
                    return self._finish(GateDecision(applied.acceptance, "accepted_hold", actions, applied, "PAUSED"))
                rotation_delta = (Rotation.from_quat(result.next_tcp_pose.quaternion_xyzw) *
                    Rotation.from_quat(source.robot_state.message.payload.tcp_pose.quaternion_xyzw).inv()).magnitude()
                if rotation_delta > 1e-8:
                    raise ValueError("source_orientation_changed")
                kwargs = dict(candidate_pose=result.next_tcp_pose, measured_state=measured_view(state.message.payload),
                              u_xy=joystick.message.payload.u_xy, dt_s=result.horizon_s, configuration=config)
                for report in (basic_step_constraints(**kwargs), self._checker(**kwargs)):
                    if not report.feasible:
                        raise ValueError("host_constraint: " + ", ".join(report.violation_reasons))
            except Exception as exc:
                failed_at = coordinator._now()
                if failed_at >= source.expires_ns:
                    return self._discard(result.result_id, "snapshot_expired", failed_at)
                return self._reject_current(result.result_id, str(exc), bundle, failed_at)
            # Check time again after host work, before action; never renew expiry.
            checked = coordinator._now()
            if checked >= source.expires_ns:
                return self._discard(result.result_id, "snapshot_expired", checked)
            try:
                coordinator._inputs(bundle, checked)
            except SnapshotError as exc:
                return self._reject_current(result.result_id, str(exc), bundle, checked)
            try:
                target = AcceptedTarget(message, checked, checked + _duration(result.horizon_s),
                                        joystick.message.payload.u_xy, state.message)
                self._adapter.apply(target)
            except Exception as exc:
                return self._reject_current(result.result_id, f"adapter_failure: {exc}",
                                            bundle, checked, fault=True)
            applied = self._accepted(message, "accepted_move", checked)
            self._last_eligible_result_ns = checked
            return self._finish(GateDecision(applied.acceptance, "accepted_move", ("apply",), applied))

    def _malformed(self, data, error):
        """Only causal host allocations can make malformed input a current failure.

        Do not repair malformed poses/statuses. Inspect a minimal typed identity
        solely to choose discard versus local stop, never to authorize motion.
        """
        raw = to_dict(data) if type(data) is Message else data
        payload = raw.get("payload", {}) if type(raw) is dict else {}
        meta = raw.get("meta", {}) if type(raw) is dict else {}
        if type(payload) is not dict: payload = {}
        if type(meta) is not dict: meta = {}
        result_id = payload.get("result_id")
        if type(result_id) is not str or not result_id.strip(): result_id = "unparsed-" + str(uuid4())
        coordinator = self._coordinator
        with coordinator.command_transaction() as bundle:
            now = coordinator._now()
            dispatch = coordinator._dispatches.get(payload.get("snapshot_id")) if type(payload.get("snapshot_id")) is str else None
            causal = (type(raw) is dict and raw.get("message_type") == "ControlResult" and
                      meta.get("session_id") == coordinator._store.session_id and dispatch is not None and
                      type(payload.get("control_epoch")) is int and
                      payload["control_epoch"] == coordinator.control_epoch == dispatch.inputs.snapshot.control_epoch and
                      type(payload.get("command_seq")) is int and payload["command_seq"] == dispatch.command_seq and
                      dispatch.command_seq > self._last_sequence and result_id not in self._accepted_results and
                      dispatch.inputs.snapshot.snapshot_seq > self._last_move_snapshot and
                      dispatch.inputs.bundle is bundle and now < dispatch.expires_ns and
                      self._enabled and self._authority_epoch == coordinator.control_epoch)
            if causal:
                return self._reject_current(result_id, "invalid_result: " + error, bundle, now)
            return self._discard(result_id, "invalid_result: " + error, now)
