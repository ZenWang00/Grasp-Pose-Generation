"""Coordinator-owned lifecycle, local withdrawal, watchdogs and recovery.

LifecycleCoordinator is the only caller of SnapshotCoordinator.set_control_epoch()
and CommandGate.set_authority()/local_hold()/local_stop(). Workers, activation
callbacks and supervisor signals never grant motion: every path to ACTIVE needs
a fresh neutral-then-nonzero engagement observed by this object while READY.
Adapter calls are submissions; nothing here claims observed physical stopping.

Lock order is lifecycle -> coordinator -> context. Registry preflight
(CommandGate.submit) and numerical preparation (ActiveBundleStore.prepare) run
without the lifecycle lock, so withdrawal never waits for them.
"""

from dataclasses import dataclass
from enum import Enum
from threading import RLock
from typing import Callable, Protocol
from uuid import uuid4

from ssc_core import Message

from .activation import ActivationOutcome, ActivationRequest, ActiveBundleStore, InitializationAttempt, _text
from .gate import CommandGate, GateDecision
from .snapshots import DispatchRecord, SnapshotCoordinator, SnapshotError, TimedInput, _duration

# offline-reference-v1 section 2: eligible-result liveness watchdog.
DEFAULT_LIVENESS_TIMEOUT_S = 0.2

# Recovery causes recorded for a PAUSED episode (Runtime Lifecycle section 6).
RELEASE, INPUT, COMPUTATION, HOST_REJECTION, CONTEXT, CONTROLLER = (
    "release", "input", "computation", "host_rejection", "context", "controller")

_INPUT_REJECTIONS = ("stale_input", "missing_input", "motion_input_disabled", "frame_mismatch")
_DISPATCH_CAUSES = {
    "stale_input": INPUT, "missing_input": INPUT, "frame_mismatch": INPUT,
    "no_active_bundle": CONTEXT, "superseded_bundle": CONTEXT,
    "budget_mismatch": CONTEXT, "invalid_duration": CONTEXT,
}


class LifecycleState(str, Enum):
    UNINITIALIZED = "UNINITIALIZED"
    VALIDATING = "VALIDATING"
    READY = "READY"
    ACTIVE = "ACTIVE"
    PAUSED = "PAUSED"
    FAULT = "FAULT"


UNINITIALIZED, VALIDATING, READY, ACTIVE, PAUSED, FAULT = LifecycleState


class LifecycleError(RuntimeError):
    """A trusted lifecycle command was refused; the refusal is also recorded."""

    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


class WorkerCanceller(Protocol):
    """Best-effort, nonblocking request to drop work issued at or before an epoch.

    Epoch validation at the gate remains necessary; a failure here is recorded,
    never awaited, and never grants or withdraws authority by itself.
    """

    def cancel(self, control_epoch: int, reason: str) -> None: ...


@dataclass(frozen=True, slots=True, eq=False)
class SessionRuntime:
    """Trusted per-session components created by the host session factory."""

    store: ActiveBundleStore
    coordinator: SnapshotCoordinator
    gate: CommandGate


@dataclass(frozen=True, slots=True)
class LifecycleRecord:
    """Local evidence of one lifecycle decision. Not observed physical motion."""

    index: int
    kind: str
    event: str
    reason: str
    session_id: str
    at_ns: int | None
    state_before: LifecycleState
    state_after: LifecycleState
    epoch_before: int
    epoch_after: int
    actions: tuple[str, ...] = ()
    hold_command_seq: int | None = None
    adapter_error: str | None = None
    detail: str | None = None
    observed_execution: None = None

    def to_dict(self) -> dict:
        return {"index": self.index, "kind": self.kind, "event": self.event, "reason": self.reason,
                "session_id": self.session_id, "at_ns": self.at_ns,
                "state_before": self.state_before.value, "state_after": self.state_after.value,
                "epoch_before": self.epoch_before, "epoch_after": self.epoch_after,
                "actions": list(self.actions), "hold_command_seq": self.hold_command_seq,
                "adapter_error": self.adapter_error, "detail": self.detail,
                "observed_execution": self.observed_execution}


def _neutral(sample: TimedInput) -> bool:
    return not SnapshotCoordinator._enabled(sample)


def _rejection_cause(reason: str) -> str:
    if reason.startswith("host_constraint"):
        return HOST_REJECTION
    if reason.startswith(_INPUT_REJECTIONS):
        return INPUT
    # Invalid output, identity mismatch, missed solve budget or unknown reason:
    # treat as computation so recovery also needs a supervisor readiness signal.
    return COMPUTATION


class LifecycleCoordinator:
    """One robot-side lifecycle across sessions; sessions come from a trusted factory.

    The caller's event loop processes, for equal local timestamps: withdrawal
    events (release arrives through on_input; invalidation, faults, restarts),
    then fresh input/state, then submit_result(), then tick(), then
    dispatch_cycle(). These are deterministic settings, not OS timing claims.
    """

    def __init__(self, session_factory: Callable[[str], SessionRuntime], *, session_id: str,
                 worker_canceller: WorkerCanceller,
                 liveness_timeout_s: float = DEFAULT_LIVENESS_TIMEOUT_S):
        if not callable(session_factory):
            raise TypeError("session_factory must be callable")
        if not callable(getattr(worker_canceller, "cancel", None)):
            raise TypeError("worker_canceller must provide cancel(control_epoch, reason)")
        self._factory, self._canceller = session_factory, worker_canceller
        self._liveness_ns = _duration(liveness_timeout_s)
        self._lock = RLock()
        self._records: list[LifecycleRecord] = []
        self._retired: list[SessionRuntime] = []
        self._used_sessions: set[str] = set()
        self._state = UNINITIALIZED
        self._controller_ready = self._computation_ready = self._fault_cleared = False
        self._fault_reason: str | None = None
        self._runtime = self._create(session_id)
        self._reset_session_state()

    # ----------------------------------------------------------------- views
    @property
    def state(self) -> LifecycleState:
        with self._lock:
            return self._state

    @property
    def runtime(self) -> SessionRuntime:
        with self._lock:
            return self._runtime

    @property
    def session_id(self) -> str:
        with self._lock:
            return self._runtime.store.session_id

    @property
    def control_epoch(self) -> int:
        with self._lock:
            return self._runtime.coordinator.control_epoch

    @property
    def records(self) -> tuple[LifecycleRecord, ...]:
        with self._lock:
            return tuple(self._records)

    @property
    def retired_sessions(self) -> tuple[SessionRuntime, ...]:
        with self._lock:
            return tuple(self._retired)

    @property
    def outstanding(self) -> tuple[DispatchRecord, ...]:
        with self._lock:
            return tuple(self._outstanding.values())

    @property
    def controller_ready(self) -> bool:
        with self._lock:
            return self._controller_ready

    @property
    def computation_ready(self) -> bool:
        with self._lock:
            return self._computation_ready

    @property
    def fault_cleared(self) -> bool:
        with self._lock:
            return self._fault_cleared

    @property
    def pause_causes(self) -> frozenset[str]:
        with self._lock:
            return frozenset(self._pause_causes)

    @property
    def liveness_deadline_ns(self) -> int | None:
        """Local time at which ACTIVE withdraws without an eligible result."""
        with self._lock:
            return self._liveness_deadline() if self._state is ACTIVE else None

    # --------------------------------------------------------- session setup
    def _create(self, session_id: str) -> SessionRuntime:
        _text(session_id, "session_id")
        if session_id in self._used_sessions:
            raise LifecycleError("session_reused", "a session ID can never be reused")
        runtime = self._factory(session_id)
        if type(runtime) is not SessionRuntime:
            raise TypeError("session_factory must return SessionRuntime")
        store, coordinator, gate = runtime.store, runtime.coordinator, runtime.gate
        if (type(store) is not ActiveBundleStore or type(coordinator) is not SnapshotCoordinator or
                type(gate) is not CommandGate):
            raise TypeError("SessionRuntime contains untrusted component types")
        if coordinator._store is not store or gate._coordinator is not coordinator:
            raise ValueError("session components are not wired to each other")
        if store.session_id != session_id:
            raise ValueError("session factory returned a different session ID")
        if (coordinator.control_epoch != 0 or coordinator._snapshot_seq or coordinator._command_seq or
                gate._enabled or store.acquire() is not None):
            raise ValueError("a new session needs fresh counters, no authority and no active context")
        self._used_sessions.add(session_id)
        return runtime

    def _reset_session_state(self):
        self._attempt: InitializationAttempt | None = None
        self._engaged_ns: int | None = None
        self._engaged_bundle = None
        self._armed_seq: int | None = None
        self._ready_seen_seq: int | None = None
        self._withdrawal_input_seq = -1
        self._pause_causes: set[str] = set()
        self._outstanding: dict[str, DispatchRecord] = {}

    # --------------------------------------------------------------- helpers
    def _now(self) -> int:
        return self._runtime.coordinator._now()

    def _latest(self, kind: str) -> TimedInput | None:
        return self._runtime.coordinator._latest.get(kind)

    def _fresh(self, sample, timeout, now) -> bool:
        if sample is None:
            return False
        try:
            self._runtime.coordinator._fresh(sample, timeout, now)
            return True
        except SnapshotError as exc:
            if exc.code == "invalid_clock":
                raise
            return False

    def _log(self, kind, event, reason, *, before, epoch_before, at_ns, actions=(),
             hold_command_seq=None, adapter_error=None, detail=None):
        record = LifecycleRecord(len(self._records), kind, event, reason, self._runtime.store.session_id,
                                 at_ns, before, self._state, epoch_before,
                                 self._runtime.coordinator.control_epoch, tuple(actions),
                                 hold_command_seq, adapter_error, detail)
        self._records.append(record)
        return record

    def _transition(self, to, event, reason, at_ns, *, detail=None):
        before, epoch = self._state, self._runtime.coordinator.control_epoch
        self._state = to
        return self._log("transition", event, reason, before=before, epoch_before=epoch,
                         at_ns=at_ns, detail=detail)

    def _note(self, kind, event, reason, at_ns, *, detail=None):
        return self._log(kind, event, reason, before=self._state,
                         epoch_before=self._runtime.coordinator.control_epoch, at_ns=at_ns, detail=detail)

    def _cancel_work(self, epoch, reason):
        try:
            self._canceller.cancel(epoch, reason)
            return None
        except Exception as exc:  # best effort: epoch validation still protects
            return f"worker_cancel: {type(exc).__name__}: {exc}"

    def _guard(self, operation, *args):
        """Run under the lifecycle lock; a failed local clock ends the session."""
        try:
            return operation(*args)
        except SnapshotError as exc:
            if exc.code != "invalid_clock":
                raise
            self._clock_failure("clock_failure", str(exc))
            return None

    def _latch(self, reason):
        # Clearance needs a controller report received after this latch.
        self._fault_cleared = False
        self._fault_reason = reason
        self._controller_ready = False

    # ------------------------------------------------------------ withdrawal
    def _withdraw(self, event, reason, to, *, causes=(), stop=False, gate_decision=None):
        """Withdraw authority: epoch, gate interlock, local hold/stop, worker cancel.

        With gate_decision the gate has already cancelled and held/stopped on its
        current-failure path; the lifecycle completes the withdrawal around it.
        A failed hold/stop always latches FAULT.
        """
        runtime = self._runtime
        coordinator, gate = runtime.coordinator, runtime.gate
        before, old = self._state, coordinator.control_epoch
        at_ns = self._now()
        actions = []
        with coordinator.command_transaction():
            if gate_decision is not None:
                actions += ["gate_local_hold", *gate_decision.actions]
            coordinator.set_control_epoch(old + 1)
            gate.set_authority(control_epoch=old + 1, motion_enabled=False,
                               controller_ready=self._controller_ready)
            actions += ["withdraw_authority", "increment_epoch"]
            latest = self._latest("JoystickSample")
            self._withdrawal_input_seq = latest.message.meta.sequence if latest else -1
            decision = gate_decision
            if decision is None:
                decision = gate.local_stop(reason) if stop else gate.local_hold(reason)
                actions += ["allocate_local_hold", *decision.actions]
        cancel_error = self._cancel_work(old, reason)
        actions.append("cancel_worker")
        self._engaged_ns = self._engaged_bundle = self._armed_seq = self._ready_seen_seq = None
        self._outstanding.clear()
        failed = decision.disposition == "adapter_failure"
        if failed and to is not FAULT:
            to = FAULT
            reason = f"{reason}; local hold/stop request failed"
        if to is FAULT and before is not FAULT:
            self._latch(reason)
        self._state = to
        self._pause_causes = set(causes) if to is PAUSED else set()
        applied = decision.applied
        return self._log("withdrawal", event, reason, before=before, epoch_before=old, at_ns=at_ns,
                         actions=actions,
                         hold_command_seq=applied.result.payload.command_seq if applied else None,
                         adapter_error=decision.adapter_error, detail=cancel_error)

    def _clock_failure(self, event, detail):
        """Freshness is untrustworthy: cancel and stop directly, then a new session."""
        runtime = self._runtime
        coordinator, gate = runtime.coordinator, runtime.gate
        before, old = self._state, coordinator.control_epoch
        with coordinator._lock:
            coordinator.set_control_epoch(old + 1)  # does not read the failed clock
            gate.set_authority(control_epoch=old + 1, motion_enabled=False, controller_ready=False)
        actions, errors = ["withdraw_authority", "increment_epoch"], []
        for name in ("cancel", "stop"):
            actions.append(name)
            try:
                getattr(gate._adapter, name)()
            except Exception as exc:
                errors.append(f"{name}: {type(exc).__name__}: {exc}")
        cancel_error = self._cancel_work(old, event)
        actions.append("cancel_worker")
        to = FAULT if before is FAULT or errors else UNINITIALIZED
        if to is FAULT and before is not FAULT:
            self._latch(f"{event}; stop request failed")
        self._state = to
        self._log("withdrawal", event, detail, before=before, epoch_before=old, at_ns=None,
                  actions=actions, adapter_error="; ".join(errors) or None, detail=cancel_error)
        self._replace_session(event, before=to)

    def _replace_session(self, event, *, before, new_session_id=None):
        old_runtime = self._runtime
        old_session, old_epoch = old_runtime.store.session_id, old_runtime.coordinator.control_epoch
        old_runtime.store.invalidate(event)
        try:
            runtime = self._create(new_session_id or str(uuid4()))
        except Exception:
            if self._state is not FAULT:
                self._latch(f"{event}; session factory failed")
                self._state = FAULT
            raise
        self._retired.append(old_runtime)
        self._runtime = runtime
        self._reset_session_state()
        try:
            at_ns = runtime.coordinator._now()
        except SnapshotError:
            at_ns = None
        return self._log("session", event, "new session allocated after withdrawal", before=before,
                         epoch_before=old_epoch, at_ns=at_ns,
                         detail=f"previous_session={old_session}")

    # ------------------------------------------------------------- inputs
    def on_input(self, data: dict | Message) -> TimedInput | None:
        """Local input path for trusted joystick/measured-state adapters.

        Release/disable withdraws immediately, without awaiting a cycle or a
        worker. Rejected or replayed samples are recorded and change nothing.
        """
        runtime = self._runtime
        try:
            timed = runtime.coordinator.receive(data)
        except SnapshotError as exc:
            with self._lock:
                if runtime is self._runtime:
                    if exc.code == "invalid_clock":
                        self._clock_failure("clock_failure", str(exc))
                    else:
                        self._note("discard", "input_rejected", exc.code, None, detail=str(exc))
            return None
        except (ValueError, TypeError) as exc:
            with self._lock:
                self._note("discard", "input_rejected", "invalid_input", None, detail=str(exc))
            return None
        with self._lock:
            if runtime is self._runtime:
                self._guard(self._process_inputs)
        return timed

    def _process_inputs(self):
        now = self._now()
        if self._state is ACTIVE:
            joystick = self._latest("JoystickSample")
            if joystick is not None and _neutral(joystick):
                reason = "joystick_released" if joystick.message.payload.enabled else "joystick_disabled"
                self._withdraw("release", reason, PAUSED, causes=(RELEASE,))
            else:
                self._check_active(now, sources_only=True)
        elif self._state is READY:
            self._ready_input(now)
        elif self._state is PAUSED:
            self._try_recover(now)

    def _ready_input(self, now):
        bundle = self._runtime.store.acquire()
        joystick = self._latest("JoystickSample")
        if bundle is None or joystick is None:
            return
        sequence = joystick.message.meta.sequence
        if self._ready_seen_seq is not None and sequence <= self._ready_seen_seq:
            return
        self._ready_seen_seq = sequence
        config = bundle.resolved.configuration
        fresh = self._fresh(joystick, config.joystick_timeout_s, now)
        if _neutral(joystick):
            self._armed_seq = sequence if fresh else None
            return
        if self._armed_seq is None:
            return  # a held or unarmed nonzero input never engages
        refusals = []
        if not fresh:
            refusals.append("stale_joystick")
        if not self._fresh(self._latest("RobotState"), config.robot_state_timeout_s, now):
            refusals.append("stale_state")
        elif self._runtime.gate._feedback(bundle, now) is None:
            refusals.append("ineligible_feedback")
        if not self._controller_ready:
            refusals.append("controller_not_ready")
        if not self._computation_ready:
            refusals.append("computation_not_ready")
        coordinator, gate = self._runtime.coordinator, self._runtime.gate
        with coordinator.command_transaction() as current:
            if current is not bundle:
                refusals.append("context_changed")
            if refusals:
                self._armed_seq = None
                self._note("refusal", "engagement_refused", ",".join(refusals), now)
                return
            gate.set_authority(control_epoch=coordinator.control_epoch, motion_enabled=True,
                               controller_ready=True)
        self._engaged_ns, self._engaged_bundle = now, bundle
        self._outstanding.clear()
        self._transition(ACTIVE, "engagement", "fresh engagement after neutral input", now,
                         detail=f"joystick_seq={sequence}")

    def _enter_ready(self, event, reason, now):
        self._transition(READY, event, reason, now)
        bundle = self._runtime.store.acquire()
        joystick = self._latest("JoystickSample")
        self._ready_seen_seq = joystick.message.meta.sequence if joystick else None
        self._armed_seq = (joystick.message.meta.sequence if joystick and _neutral(joystick) and
                           self._fresh(joystick, bundle.resolved.configuration.joystick_timeout_s, now)
                           else None)

    def _try_recover(self, now):
        """PAUSED -> READY: valid bundle, fresh inputs, newer neutral, restored causes."""
        bundle = self._runtime.store.acquire()
        if bundle is None:
            return  # invalid context recovers only through VALIDATING
        try:
            joystick, _ = self._runtime.coordinator.current_inputs()
        except SnapshotError as exc:
            if exc.code == "invalid_clock":
                raise
            return
        if not _neutral(joystick) or joystick.message.meta.sequence <= self._withdrawal_input_seq:
            return
        if COMPUTATION in self._pause_causes and not self._computation_ready:
            return
        if self._runtime.gate._feedback(bundle, now) is None:
            return
        causes = ",".join(sorted(self._pause_causes)) or "none"
        self._pause_causes = set()
        self._enter_ready("recovery", f"restored after {causes}; neutral observed", now)

    def _check_active(self, now, *, sources_only=False):
        bundle = self._engaged_bundle
        if not self._runtime.store.is_current(bundle):
            self._withdraw("context_invalidated", "active context is no longer current", PAUSED,
                           causes=(CONTEXT,))
            return
        config = bundle.resolved.configuration
        if not self._fresh(self._latest("JoystickSample"), config.joystick_timeout_s, now):
            self._withdraw("stale_input", "stale_joystick", PAUSED, causes=(INPUT,))
            return
        if not self._fresh(self._latest("RobotState"), config.robot_state_timeout_s, now):
            self._withdraw("stale_input", "stale_state", PAUSED, causes=(INPUT,))
            return
        if sources_only:
            return
        for dispatch in self._outstanding.values():
            if now >= dispatch.expires_ns:
                self._withdraw("missed_result_deadline", "missed_result_deadline", PAUSED,
                               causes=(COMPUTATION,))
                return
        if now >= self._liveness_deadline():
            self._withdraw("liveness_watchdog", "result_liveness_expired", PAUSED, causes=(COMPUTATION,))

    def _liveness_deadline(self):
        last = self._runtime.gate.last_eligible_result_ns
        anchor = self._engaged_ns if last is None else max(self._engaged_ns, last)
        return anchor + self._liveness_ns

    # -------------------------------------------------------------- watchdog
    def tick(self) -> None:
        """Watchdogs at the current local time: sources, deadlines, liveness."""
        with self._lock:
            self._guard(self._tick)

    def _tick(self):
        now = self._now()
        if self._state is ACTIVE:
            self._check_active(now)
        elif self._state is READY:
            bundle = self._runtime.store.acquire()
            if bundle is None:
                self._withdraw("context_invalidated", "ready context is no longer current", PAUSED,
                               causes=(CONTEXT,))
                return
            config = bundle.resolved.configuration
            if not self._fresh(self._latest("JoystickSample"), config.joystick_timeout_s, now):
                self._withdraw("required_input_unavailable", "stale_joystick", PAUSED, causes=(INPUT,))
            elif not self._fresh(self._latest("RobotState"), config.robot_state_timeout_s, now):
                self._withdraw("required_input_unavailable", "stale_state", PAUSED, causes=(INPUT,))
        elif self._state is PAUSED:
            self._try_recover(now)

    # -------------------------------------------------------------- dispatch
    def dispatch_cycle(self) -> DispatchRecord | None:
        """Freeze and dispatch one snapshot, only while ACTIVE."""
        with self._lock:
            return self._guard(self._dispatch)

    def _dispatch(self):
        if self._state is not ACTIVE:
            return None
        coordinator = self._runtime.coordinator
        try:
            record = coordinator.dispatch(coordinator.assemble())
        except SnapshotError as exc:
            if exc.code == "invalid_clock":
                raise
            if exc.code == "motion_disabled":
                self._withdraw("release", "joystick_released", PAUSED, causes=(RELEASE,))
            else:
                cause = _DISPATCH_CAUSES.get(exc.code, COMPUTATION)
                self._withdraw("dispatch_failure", str(exc), PAUSED, causes=(cause,))
            return None
        except Exception as exc:
            self._withdraw("dispatch_failure", f"{type(exc).__name__}: {exc}", PAUSED,
                           causes=(COMPUTATION,))
            raise
        self._outstanding[record.inputs.snapshot.snapshot_id] = record
        return record

    # --------------------------------------------------------------- results
    def submit_result(self, data: dict | Message, *, weights: dict | Message | None = None) -> GateDecision | None:
        """Worker result entry point. Gate preflight runs without the lifecycle lock."""
        runtime = self._runtime
        epoch = runtime.coordinator.control_epoch
        try:
            decision = runtime.gate.submit(data, weights=weights)
        except SnapshotError as exc:
            if exc.code != "invalid_clock":
                raise
            with self._lock:
                if runtime is self._runtime:
                    self._clock_failure("clock_failure", str(exc))
            return None
        with self._lock:
            if runtime is self._runtime:
                self._guard(self._after_decision, data, decision, epoch)
        return decision

    def _after_decision(self, data, decision, epoch_at_submit):
        snapshot_id = None
        if type(data) is Message:
            snapshot_id = getattr(data.payload, "snapshot_id", None)
        elif type(data) is dict and type(data.get("payload")) is dict:
            snapshot_id = data["payload"].get("snapshot_id")
        if decision.disposition != "discarded" and type(snapshot_id) is str:
            self._outstanding.pop(snapshot_id, None)
        if decision.disposition == "accepted_move":
            accepted_seq = decision.applied.source.snapshot.snapshot_seq
            for key, dispatch in tuple(self._outstanding.items()):
                if dispatch.inputs.snapshot.snapshot_seq <= accepted_seq:
                    del self._outstanding[key]
        request = decision.lifecycle_request
        if request == "FAULT":
            if self._state is not FAULT:
                self._withdraw("result_fault", decision.proposal.reason, FAULT, gate_decision=decision)
        elif request == "PAUSED":
            current = self._runtime.coordinator.control_epoch == epoch_at_submit
            if self._state is ACTIVE and current:
                cause = (COMPUTATION if decision.disposition == "accepted_hold"
                         else _rejection_cause(decision.proposal.reason))
                self._withdraw("result_failure", decision.proposal.reason, PAUSED,
                               causes=(cause,), gate_decision=decision)

    # --------------------------------------------------------- initialization
    def begin_initialization(self, request: ActivationRequest) -> InitializationAttempt:
        """Start a motion-disabled initialization; withdraws first if needed."""
        with self._lock:
            return self._guard(self._begin, request)

    def _begin(self, request):
        now = self._now()
        if self._state is FAULT:
            self._note("refusal", "initialization_refused", "fault_latched", now)
            raise LifecycleError("fault_latched", "rearm after fault clearance before initialization")
        if self._state is ACTIVE:
            self._withdraw("context_replacement", "initialization_requested", PAUSED, causes=(CONTEXT,))
        elif self._state is READY:
            self._withdraw("context_replacement", "initialization_requested", VALIDATING)
        attempt = self._runtime.store.begin(request)
        self._attempt = attempt
        if self._state is VALIDATING:  # READY replacement or a superseding attempt
            self._note("note", "initialization_started", attempt.attempt_id, now)
        else:
            self._transition(VALIDATING, "initialization_started", attempt.attempt_id, now)
        return attempt

    def prepare_initialization(self, attempt: InitializationAttempt) -> ActivationOutcome:
        """Run validation outside the lifecycle lock; host failures latch FAULT."""
        runtime = self._runtime
        with self._lock:
            if attempt is not self._attempt or self._state is not VALIDATING:
                return ActivationOutcome("superseded", getattr(attempt, "attempt_id", None),
                                         "superseded_attempt", "lifecycle is not validating this attempt")
        try:
            return runtime.store.prepare(attempt)
        except Exception as exc:
            with self._lock:
                if runtime is self._runtime and attempt is self._attempt and self._state is VALIDATING:
                    self._attempt = None
                    self._guard(lambda: self._withdraw(
                        "host_failure", f"unrecoverable_host_condition: {type(exc).__name__}: {exc}",
                        FAULT, stop=True))
            raise

    def complete_initialization(self, outcome: ActivationOutcome) -> ActivationOutcome:
        """Publish a prepared current attempt. Never engages motion."""
        with self._lock:
            return self._guard(self._complete, outcome)

    def _complete(self, outcome):
        now = self._now()
        attempt = self._attempt
        if (self._state is not VALIDATING or attempt is None or type(outcome) is not ActivationOutcome or
                outcome.attempt_id != attempt.attempt_id):
            self._note("discard", "initialization_result_discarded", "superseded_attempt", now)
            return ActivationOutcome("superseded", getattr(outcome, "attempt_id", None),
                                     "superseded_attempt", "lifecycle is not waiting for this attempt")
        if outcome.status == "prepared":
            published = self._runtime.store.publish(outcome)
            if published.status == "activated":
                self._attempt = None
                self._enter_ready("activation", "compatible bundle activated", now)
                return published
            outcome = published
        self._attempt = None
        self._runtime.store.invalidate(f"initialization {outcome.status}: {outcome.code}")
        self._transition(PAUSED, "validation_rejected", outcome.code, now, detail=outcome.diagnostic)
        self._pause_causes = {CONTEXT}
        return outcome

    def initialize(self, request: ActivationRequest) -> ActivationOutcome:
        attempt = self.begin_initialization(request)
        return self.complete_initialization(self.prepare_initialization(attempt))

    def cancel_initialization(self, reason: str) -> None:
        _text(reason, "reason")
        with self._lock:
            self._guard(self._cancel_initialization, reason)

    def _cancel_initialization(self, reason):
        now = self._now()
        if self._state is not VALIDATING:
            return
        self._attempt = None
        self._runtime.store.invalidate(reason)
        self._transition(PAUSED, "initialization_cancelled", reason, now)
        self._pause_causes = {CONTEXT}

    # ------------------------------------------------------ invalidation/faults
    def invalidate_context(self, reason: str) -> None:
        """Task/scene/semantic/feature/configuration invalidation."""
        _text(reason, "reason")
        with self._lock:
            self._guard(self._invalidate, reason)

    def _invalidate(self, reason):
        now = self._now()
        if self._state in (ACTIVE, READY):
            self._withdraw("context_invalidated", reason, PAUSED, causes=(CONTEXT,))
            self._runtime.store.invalidate(reason)
        elif self._state is VALIDATING:
            self._attempt = None
            self._runtime.store.invalidate(reason)
            self._transition(PAUSED, "context_invalidated", reason, now)
            self._pause_causes = {CONTEXT}
        else:
            self._runtime.store.invalidate(reason)
            if self._state is PAUSED:
                self._pause_causes.add(CONTEXT)
            self._note("note", "context_invalidated", reason, now)

    def controller_status(self, *, ready: bool, fault: str | None = None) -> None:
        """Trusted controller status. A fault latches FAULT from any state."""
        if type(ready) is not bool:
            raise TypeError("ready must be boolean")
        if fault is not None:
            _text(fault, "fault")
        with self._lock:
            self._guard(self._controller, ready, fault)

    def _controller(self, ready, fault):
        now = self._now()
        if fault is not None:
            self._controller_ready = False
            if self._state is FAULT:
                self._fault_cleared = False
                self._note("signal", "controller_fault", fault, now)
            else:
                self._withdraw("controller_fault", fault, FAULT, stop=True)
            return
        self._controller_ready = ready
        if self._state is FAULT:
            if ready:
                self._fault_cleared = True
            self._note("signal", "controller_status", "fault_cleared" if ready else "not_ready", now)
        elif not ready and self._state is ACTIVE:
            self._withdraw("controller_not_ready", "controller_not_ready", PAUSED, causes=(CONTROLLER,))

    def computation_status(self, *, ready: bool, reason: str) -> None:
        """Trusted solver-supervisor readiness, issued after motion-disabled preparation."""
        if type(ready) is not bool:
            raise TypeError("ready must be boolean")
        _text(reason, "reason")
        with self._lock:
            self._guard(self._computation, ready, reason)

    def _computation(self, ready, reason):
        now = self._now()
        self._computation_ready = ready
        self._note("signal", "computation_status", "ready" if ready else "unavailable", now, detail=reason)
        if not ready and self._state is ACTIVE:
            self._withdraw("computation_unavailable", reason, PAUSED, causes=(COMPUTATION,))
        elif ready and self._state is PAUSED:
            self._try_recover(now)

    def rearm(self, reason: str) -> LifecycleRecord:
        """Explicit operator rearm; only after a controller report clears the fault."""
        _text(reason, "reason")
        with self._lock:
            return self._guard(self._rearm, reason)

    def _rearm(self, reason):
        now = self._now()
        if self._state is not FAULT:
            self._note("refusal", "rearm_refused", "not_faulted", now)
            raise LifecycleError("not_faulted", "rearm applies only to a latched FAULT")
        if not self._fault_cleared:
            self._note("refusal", "rearm_refused", "fault_not_cleared", now, detail=self._fault_reason)
            raise LifecycleError("fault_not_cleared", "controller has not reported the fault cleared")
        self._runtime.store.invalidate("rearm requires reinitialization")
        self._fault_cleared, self._fault_reason = False, None
        self._reset_session_state()
        return self._transition(UNINITIALIZED, "rearm", reason, now)

    # ---------------------------------------------------------------- session
    def restart_session(self, reason: str, *, new_session_id: str | None = None) -> str:
        """Withdraw, then retire all session records and counters. FAULT stays latched."""
        _text(reason, "reason")
        with self._lock:
            self._guard(self._restart, reason, new_session_id)
            return self._runtime.store.session_id

    def _restart(self, reason, new_session_id):
        if new_session_id is not None:
            _text(new_session_id, "new_session_id")
            if new_session_id in self._used_sessions:
                self._note("refusal", "restart_refused", "session_reused", self._now())
                raise LifecycleError("session_reused", "a session ID can never be reused")
        was_fault = self._state is FAULT
        record = self._withdraw("session_restart", reason, FAULT if was_fault else UNINITIALIZED,
                                stop=was_fault)
        self._replace_session("session_restart", before=record.state_after, new_session_id=new_session_id)

    def clock_domain_reset(self, reason: str) -> str:
        """A clock-domain epoch reset starts a new session; freshness cannot be reused."""
        _text(reason, "reason")
        with self._lock:
            self._clock_failure("clock_domain_reset", reason)
            return self._runtime.store.session_id
