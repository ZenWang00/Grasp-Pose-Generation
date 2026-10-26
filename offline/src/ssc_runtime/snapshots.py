"""Immutable cycle inputs and local dispatch evidence, never command authority.

Only trusted local adapters feed this coordinator. Their acquisition stamps
must already use its declared monotonic domain; no receipt-based clock mapping
is inferred. The future gate owns acceptance and physical application.
"""

from collections import deque
from dataclasses import dataclass
from decimal import Decimal
from threading import RLock
from typing import Callable
from uuid import uuid4

from ssc_core import (
    CommandAcceptanceRecord, ControlSnapshot, Message,
    NUMERIC_OFFLINE_V1, ResolvedSnapshot, TimePoint, parse_message, parse_record,
    to_dict,
)

from .activation import ActiveBundle, ActiveBundleStore, _text

NS = 1_000_000_000
HISTORY_LIMIT = 32


class SnapshotError(ValueError):
    """Rejected input or local operation with a stable diagnostic code."""

    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


def _require(condition, code, detail):
    if not condition:
        raise SnapshotError(code, detail)


def _counter(value, name):
    _require(type(value) is int and value >= 0, "invalid_counter", name)
    return value


def _duration(value):
    # Config has already been parsed. Reject sub-nanosecond settings instead
    # of rounding up an authority budget or silently shortening a source age.
    ns = Decimal(str(value)) * NS
    _require(ns > 0 and ns == ns.to_integral_value(), "invalid_duration",
             "duration must be a positive whole number of nanoseconds")
    return int(ns)


@dataclass(frozen=True, slots=True)
class TimedInput:
    message: Message
    sampled_ns: int
    received_ns: int


@dataclass(frozen=True, slots=True)
class AcceptedCommand:
    """Trusted adapter bookkeeping, not observed physical motion."""

    result: Message
    acceptance: CommandAcceptanceRecord
    source: "FrozenSnapshot | None"


@dataclass(frozen=True, slots=True, eq=False)
class FrozenSnapshot:
    session_id: str
    clock_id: str
    bundle: ActiveBundle
    resolved: ResolvedSnapshot
    joystick: TimedInput
    robot_state: TimedInput
    history: tuple[TimedInput, ...]
    previous_applied: AcceptedCommand | None
    created_ns: int
    expires_ns: int
    motion_eligible: bool

    @property
    def snapshot(self) -> ControlSnapshot:
        return self.resolved.snapshot


@dataclass(frozen=True, slots=True, eq=False)
class DispatchRecord:
    """Worker envelope; the same object is returned for a permitted retry."""

    inputs: FrozenSnapshot
    command_seq: int
    dispatched_ns: int
    solve_deadline_ns: int
    transport_allowance_ns: int

    @property
    def expires_ns(self) -> int:
        return self.inputs.expires_ns


@dataclass(frozen=True, slots=True)
class ResultArrival:
    """Local receipt evidence only. It is not a gate acceptance decision."""

    dispatch: DispatchRecord
    result: Message
    received_ns: int
    before_expiry: bool
    before_solve_deadline: bool


class SnapshotCoordinator:
    """One session's serialized local input/dispatch bookkeeping.

    Lifecycle supplies explicit increasing epochs. Ordinary input never changes
    an epoch. Store currency is checked at each boundary but is not a lease:
    the final gate must coordinate authority, release and context atomically.
    Do not call this API while holding store.artifacts() in another thread.
    Session evidence is retained for lookup/replay; only live intent history is
    bounded (32 earlier samples). Retire the coordinator at session end.
    """

    def __init__(self, store: ActiveBundleStore, *, clock_ns: Callable[[], int],
                 clock_id: str, joystick_source_id: str, robot_state_source_id: str,
                 mapping_id: str, control_epoch: int = 0,
                 transport_allowance_ns: int = 0):
        if type(store) is not ActiveBundleStore or not callable(clock_ns):
            raise TypeError("expected ActiveBundleStore and callable local clock")
        for name, value in (("clock_id", clock_id), ("mapping_id", mapping_id),
                            ("joystick_source_id", joystick_source_id),
                            ("robot_state_source_id", robot_state_source_id)):
            _text(value, name)
        self._store, self._clock = store, clock_ns
        self._clock_id, self._mapping_id = clock_id, mapping_id
        self._sources = {"JoystickSample": joystick_source_id, "RobotState": robot_state_source_id}
        self._epoch = _counter(control_epoch, "control_epoch")
        self._transport = _counter(transport_allowance_ns, "transport_allowance_ns")
        self._lock = RLock()
        self._last_now = None
        self._clock_failed = False
        self._latest: dict[str, TimedInput] = {}
        self._history: deque[TimedInput] = deque(maxlen=HISTORY_LIMIT + 1)
        self._snapshots: dict[str, FrozenSnapshot] = {}
        self._dispatches: dict[str, DispatchRecord] = {}
        self._local_commands: dict[int, tuple[int, int]] = {}
        self._accepted: dict[str, AcceptedCommand] = {}
        self._previous: AcceptedCommand | None = None
        self._snapshot_seq = self._command_seq = 0

    @property
    def control_epoch(self):
        with self._lock:
            return self._epoch

    def set_control_epoch(self, epoch: int):
        """Mirror a lifecycle-owned cancellation; does not implement hold/stop."""
        with self._lock:
            _counter(epoch, "control_epoch")
            _require(epoch > self._epoch, "old_epoch", "epoch must increase")
            self._epoch = epoch

    def _now(self):
        value = self._clock()
        if (type(value) is not int or value < 0 or
                (self._last_now is not None and value < self._last_now)):
            self._clock_failed = True
        _require(not self._clock_failed, "invalid_clock", "local clock failed; start a new session")
        self._last_now = value
        return value

    def _stamp(self, stamp: TimePoint):
        _require(stamp.clock_id == self._clock_id, "unverifiable_clock",
                 "source acquisition must use the declared local domain")
        return stamp.sec * NS + stamp.nanosec

    def _message(self, data):
        message = parse_message(to_dict(data) if type(data) is Message else data,
                                policy=NUMERIC_OFFLINE_V1)
        _require(message.meta.session_id == self._store.session_id, "session_mismatch",
                 "message is not from this session")
        return message

    def receive(self, data: dict | Message) -> TimedInput:
        """Receive a trusted adapter sample without replacing its source stamp.

        New but stale samples remain visibly stale, preventing fallback to an
        earlier sample. Replayed/out-of-order samples cannot replace selection.
        produced_at may use a foreign clock: it is provenance, never freshness.
        """
        with self._lock:
            message = self._message(data)
            kind = message.message_type
            _require(kind in self._sources, "input_type", "expected joystick or measured state")
            _require(message.meta.source_id == self._sources[kind], "input_source", kind)
            sampled = self._stamp(message.payload.sampled_at)
            now = self._now()
            _require(sampled <= now, "future_sample", kind)
            if kind == "JoystickSample":
                _require(message.payload.mapping_id == self._mapping_id, "mapping_mismatch", kind)
            previous = self._latest.get(kind)
            if previous:
                _require(message.meta.sequence > previous.message.meta.sequence,
                         "replayed_input", kind)
                _require(sampled >= previous.sampled_ns, "reversed_sample_time", kind)
                _require(message.meta.message_id != previous.message.meta.message_id,
                         "replayed_input", "message identity was reused")
            with self._store.artifacts() as (registry, _):
                message = registry.register_message(message)
            timed = TimedInput(message, sampled, now)
            self._latest[kind] = timed
            if kind == "JoystickSample":
                self._history.append(timed)
            return timed

    def _fresh(self, sample, timeout, now):
        _require(0 <= now - sample.sampled_ns <= _duration(timeout),
                 "stale_input", sample.message.message_type)

    def _inputs(self, bundle, now):
        _require(all(kind in self._latest for kind in self._sources),
                 "missing_input", "joystick and measured state are required")
        joystick, state = self._latest["JoystickSample"], self._latest["RobotState"]
        config = bundle.resolved.configuration
        self._fresh(joystick, config.joystick_timeout_s, now)
        self._fresh(state, config.robot_state_timeout_s, now)
        _require(state.message.payload.frame_id == config.control_frame and
                 state.message.payload.tcp_frame == config.tcp_frame,
                 "frame_mismatch", "measured state differs from active configuration")
        return joystick, state

    @staticmethod
    def _enabled(joystick):
        payload = joystick.message.payload
        return payload.enabled and any(value != 0 for value in payload.u_xy)

    def current_inputs(self) -> tuple[TimedInput, TimedInput]:
        """Read fresh current joystick/state without allocating a snapshot.

        Neutral/disabled input remains visible. The return is immutable evidence
        at this instant, not a lease across release or context replacement.
        """
        with self._lock:
            now = self._now()
            bundle = self._store.acquire()
            _require(bundle is not None, "no_active_bundle", "context has not been activated")
            return self._inputs(bundle, now)

    def assemble(self) -> FrozenSnapshot:
        with self._lock:
            now = self._now()
            bundle = self._store.acquire()
            _require(bundle is not None, "no_active_bundle", "context has not been activated")
            config = bundle.resolved.configuration
            age, budget = _duration(config.max_command_age_s), _duration(config.solve_budget_s)
            _duration(config.control_period_s)
            _require(budget + self._transport <= age, "budget_mismatch",
                     "solve and transport allowances exceed original age budget")
            joystick, state = self._inputs(bundle, now)
            earliest = now - _duration(config.history_window_s)
            history = tuple(item for item in self._history
                            if item is not joystick and item.sampled_ns >= earliest)[-HISTORY_LIMIT:]
            request = bundle.attempt.request
            snapshot = parse_record(ControlSnapshot, {
                "snapshot_id": str(uuid4()), "snapshot_seq": self._snapshot_seq + 1,
                "control_epoch": self._epoch, "config_id": config.config_id,
                "task_ref": to_dict(request.task_ref), "dt_s": config.control_period_s,
                "joystick_ref": joystick.message.meta.message_id,
                "robot_state_ref": state.message.meta.message_id,
                "joystick_history_refs": [item.message.meta.message_id for item in history],
                "geometry_ref": to_dict(request.geometry_ref), "semantic_id": request.semantic_id,
                "program_refs": [to_dict(ref) for ref in request.program_refs],
                "execution_binding_refs": list(request.execution_binding_refs),
                "previous_applied_command_id": (self._previous.acceptance.applied_command_id
                                                if self._previous else None),
            }, policy=NUMERIC_OFFLINE_V1)
            resolved = ResolvedSnapshot(snapshot, bundle.resolved, joystick.message, state.message,
                                        tuple(item.message for item in history))
            frozen = FrozenSnapshot(self._store.session_id, self._clock_id, bundle, resolved,
                                    joystick, state, history, self._previous, now, now + age,
                                    self._enabled(joystick))
            _require(self._store.is_current(bundle), "superseded_bundle", "context changed during assembly")
            with self._store.artifacts() as (registry, _):
                registry.register_record(ControlSnapshot, snapshot, session_id=self._store.session_id)
            self._snapshot_seq += 1
            self._snapshots[snapshot.snapshot_id] = frozen
            return frozen

    def lookup_snapshot(self, snapshot_id: str) -> FrozenSnapshot:
        with self._lock:
            _require(snapshot_id in self._snapshots, "unknown_snapshot", snapshot_id)
            return self._snapshots[snapshot_id]

    def lookup_dispatch(self, snapshot_id: str) -> DispatchRecord:
        with self._lock:
            _require(snapshot_id in self._dispatches, "unknown_dispatch", snapshot_id)
            return self._dispatches[snapshot_id]

    def allocate_local_command_sequence(self) -> int:
        """Reserve a sequence for a downstream gate's local hold decision."""
        with self._lock:
            now = self._now()
            self._command_seq += 1
            self._local_commands[self._command_seq] = (self._epoch, now)
            return self._command_seq

    def dispatch(self, frozen: FrozenSnapshot) -> DispatchRecord:
        with self._lock:
            _require(type(frozen) is FrozenSnapshot and
                     self._snapshots.get(frozen.snapshot.snapshot_id) is frozen,
                     "foreign_snapshot", "expected this coordinator's original snapshot")
            now = self._now()
            snapshot, config = frozen.snapshot, frozen.bundle.resolved.configuration
            _require(snapshot.control_epoch == self._epoch, "old_epoch", "snapshot was canceled")
            _require(self._store.is_current(frozen.bundle), "superseded_bundle", "context is no longer current")
            _require(now < frozen.expires_ns, "snapshot_expired", snapshot.snapshot_id)
            self._fresh(frozen.joystick, config.joystick_timeout_s, now)
            self._fresh(frozen.robot_state, config.robot_state_timeout_s, now)
            latest, _ = self._inputs(frozen.bundle, now)
            _require(frozen.motion_eligible and self._enabled(latest), "motion_disabled",
                     "neutral/disabled input cannot dispatch assistance")
            previous = self._dispatches.get(snapshot.snapshot_id)
            if previous:
                _require(now < previous.solve_deadline_ns, "solve_expired", "retry cannot restart solve time")
                return previous
            deadline = min(now + _duration(config.solve_budget_s), frozen.expires_ns - self._transport)
            _require(now < deadline, "solve_expired", "no remaining solve/transport budget")
            self._command_seq += 1
            record = DispatchRecord(frozen, self._command_seq, now, deadline, self._transport)
            self._dispatches[snapshot.snapshot_id] = record
            return record

    def _result_dispatch(self, message):
        result = message.payload
        dispatch = self.lookup_dispatch(result.snapshot_id)
        source = dispatch.inputs
        _require(result.command_seq == dispatch.command_seq and
                 result.control_epoch == source.snapshot.control_epoch and
                 result.source_joystick_seq == source.joystick.message.meta.sequence and
                 result.source_robot_state_seq == source.robot_state.message.meta.sequence,
                 "result_mismatch", "result did not echo its local dispatch")
        if result.status == "move":
            _require(result.horizon_s == source.snapshot.dt_s, "result_mismatch", "horizon differs")
        return dispatch

    def observe_result(self, data: dict | Message) -> ResultArrival:
        """Stamp worker arrival locally; production/completion stamps grant nothing."""
        with self._lock:
            message = self._message(data)
            _require(message.message_type == "ControlResult", "result_type", "expected ControlResult")
            dispatch = self._result_dispatch(message)
            now = self._now()
            return ResultArrival(dispatch, message, now, now < dispatch.expires_ns,
                                 now < dispatch.solve_deadline_ns)

    def record_acceptance(self, data: dict | Message,
                          acceptance: dict | CommandAcceptanceRecord) -> AcceptedCommand | None:
        """Ingest the trusted local adapter's decision, never a worker boolean.

        This checks provenance/order only, not constraints or motion permission.
        Call only after the future gate has independently accepted/applied it.
        A rejected or merely solved proposal cannot become smoothing history.
        """
        with self._lock:
            message = self._message(data)
            _require(message.message_type == "ControlResult", "result_type", "expected ControlResult")
            record = parse_record(CommandAcceptanceRecord,
                                  to_dict(acceptance) if type(acceptance) is CommandAcceptanceRecord else acceptance,
                                  policy=NUMERIC_OFFLINE_V1)
            result = message.payload
            _require(record.result_id == result.result_id, "acceptance_mismatch", "result ID differs")
            _require(record.command_accepted == (record.applied_command_id is not None),
                     "acceptance_mismatch", "only accepted commands have an applied ID")
            checked, now = self._stamp(record.checked_at), self._now()
            _require(checked <= now, "future_acceptance", "acceptance check is in the future")
            if not record.command_accepted:
                return None
            _require(result.control_epoch == self._epoch, "old_epoch", "accepted result was canceled")
            _require(record.applied_command_id not in self._accepted, "duplicate_acceptance", "applied ID reused")
            _require(all(entry.result.payload.result_id != result.result_id for entry in self._accepted.values()),
                     "duplicate_acceptance", "result was already accepted")
            if self._previous:
                _require(result.command_seq > self._previous.result.payload.command_seq,
                         "out_of_order_acceptance", "decision must follow the last accepted command")
                _require(checked >= self._stamp(self._previous.acceptance.checked_at),
                         "out_of_order_acceptance", "acceptance time moved backwards")
            source = None
            if result.snapshot_id is not None:
                dispatch = self._result_dispatch(message)
                source = dispatch.inputs
                _require(dispatch.dispatched_ns <= checked < source.expires_ns,
                         "acceptance_expired", "acceptance is outside original dispatch validity")
            else:
                allocation = self._local_commands.get(result.command_seq)
                _require(result.status == "hold" and allocation is not None and
                         allocation[0] == self._epoch and allocation[1] <= checked,
                         "unknown_local_command", "local hold must use a reserved host sequence")
            with self._store.artifacts() as (registry, _):
                registry.register_message(message)
            entry = AcceptedCommand(message, record, source)
            self._accepted[record.applied_command_id] = entry
            self._previous = entry
            return entry

    def lookup_applied(self, applied_command_id: str) -> AcceptedCommand:
        with self._lock:
            _require(applied_command_id in self._accepted, "unknown_applied_command", applied_command_id)
            return self._accepted[applied_command_id]
