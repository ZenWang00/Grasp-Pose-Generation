"""Serialized offline coordinator with asynchronous, separately supervised solves.

Simulation time advances only in 20 ms ticks. Real solver/evaluator budgets are
independently enforced by ssc-core; no simulated rate is a hardware guarantee.
"""

from dataclasses import asdict, dataclass
import time

from ssc_core import (AppliedCommandReference, HostCheckerSpec, NUMERIC_OFFLINE_V1,
                      SupervisedOptimizer, estimate_intent, get_intent_config,
                      parse_message, parse_record, to_dict)
from .gate import CommandGate
from .lifecycle import LifecycleCoordinator, LifecycleState as S, SessionRuntime
from .plant import ControllerError, PLANT_TICK_NS, SyntheticController
from .scenarios import Fixture, MAPPING
from .snapshots import SnapshotCoordinator


@dataclass
class SimulationClock:
    now: int = 10_000_000_000
    clock_id: str = "integration-local-v1"

    def __call__(self):
        return self.now

    def stamp(self, ns=None):
        ns = self.now if ns is None else ns
        return {"clock_id": self.clock_id, "sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}


class OfflineLoop:
    def __init__(self, variant="default", *, session_id="session-integration-v1", initial_joints=(0.,) * 6):
        self.clock = SimulationClock()
        self.fixture = Fixture(variant)
        self.session_id = session_id
        self.intent_config = get_intent_config(self.fixture.intent_id)
        self.optimizer = None
        self.pending = None
        self.retirements = []
        self.preparations, self.solves, self.dispatches, self.cancellations = [], [], [], []
        self.messages, self.events = {}, []
        self.counters = {}
        self._seen_plant = self._seen_gate = self._seen_lifecycle = 0
        self.u = (0., 0.)
        self.initial_joints = initial_joints
        self.lifecycle = LifecycleCoordinator(self._factory, session_id=session_id, worker_canceller=self)
        self.capture()

    def _factory(self, session_id):
        # One archive per session. Session/clock restart is exercised separately
        # by the component suite; a new OfflineLoop starts a new run archive.
        if session_id != self.session_id:
            raise ValueError("create a new OfflineLoop for a new session")
        self.store, self.request = self.fixture.create_store(session_id)
        config = parse_record("SessionConfig", self.fixture.records["SessionConfig"], policy=NUMERIC_OFFLINE_V1)
        self.plant = SyntheticController(config, session_id=session_id,
            clock_ns=self.clock, clock_id=self.clock.clock_id, initial_joints=self.initial_joints)
        self.coordinator = SnapshotCoordinator(self.store, clock_ns=self.clock, clock_id=self.clock.clock_id,
            joystick_source_id="offline-joystick", robot_state_source_id=self.plant.source_id, mapping_id=MAPPING)
        self.gate = CommandGate(self.coordinator, self.plant)
        return SessionRuntime(self.store, self.coordinator, self.gate)

    def event(self, kind, data):
        self.events.append({"event_id": f"event-{len(self.events)}", "kind": kind, "data": data})

    def envelope(self, kind, payload):
        sequence = self.counters.get(kind, 0) + 1
        self.counters[kind] = sequence
        return parse_message({"message_type": kind, "meta": {
            "schema_version": "1.0", "session_id": self.session_id,
            "source_id": "offline-joystick" if kind == "JoystickSample" else "offline-" + kind,
            "message_id": f"{self.session_id}:{kind}:{sequence}", "sequence": sequence,
            "produced_at": self.clock.stamp()}, "payload": to_dict(payload) if not isinstance(payload, dict) else payload},
            policy=NUMERIC_OFFLINE_V1)

    def receive(self, message):
        value = self.lifecycle.on_input(message)
        if value is not None:
            self.messages[message.meta.message_id] = message
        self.capture()
        return value

    def input(self, u=(1., 0.), *, enabled=True):
        self.u = tuple(u) if enabled else (0., 0.)
        message = self.envelope("JoystickSample", {"sampled_at": self.clock.stamp(),
            "mapping_id": MAPPING, "u_xy": list(u), "enabled": enabled})
        self.receive(message)
        return message

    def feedback(self):
        message = self.plant.sample()
        if message is not None:
            self.receive(message)
        self.lifecycle.controller_status(**asdict(self.plant.status))
        self.capture()
        return message

    def prepare(self):
        if self.lifecycle.state not in (S.UNINITIALIZED, S.READY, S.PAUSED, S.VALIDATING):
            raise RuntimeError("prepare requires disabled motion")
        if self.pending is not None and not self.pending[0].done():
            raise RuntimeError("previous worker is still retiring")
        if self.optimizer is None:
            self.optimizer = SupervisedOptimizer(HostCheckerSpec("ssc_runtime.constraints:SyntheticHostChecker"))
        bundle = self.store.acquire()
        if bundle is None:
            raise RuntimeError("no activated bundle")
        report = self.optimizer.prepare(bundle.resolved, motion_disabled=True).result(10)
        self.preparations.append(asdict(report))
        self.lifecycle.computation_status(ready=report.ok, reason=report.code)
        self.capture()
        if not report.ok:
            raise RuntimeError(f"solver preparation failed: {report.code}: {report.diagnostic}")

    def initialize(self):
        self.feedback()
        self.input((0., 0.))
        outcome = self.lifecycle.initialize(self.request)
        self.capture()
        if outcome.status != "activated":
            raise RuntimeError(f"activation failed: {outcome}")
        self.prepare()
        self.input((0., 0.))
        return outcome

    def cancel(self, control_epoch, reason):
        """Lifecycle's nonblocking cancellation port; never call it back here."""
        self.cancellations.append({"at_ns": self.clock.now, "control_epoch": control_epoch, "reason": reason})
        if self.pending is not None and not self.pending[0].done() and self.optimizer is not None:
            self.retirements.append(self.optimizer.close())
            self.optimizer = None

    def dispatch(self):
        if self.pending is not None:
            raise RuntimeError("consume the previous solve before dispatch")
        if self.optimizer is None or not self.optimizer.ready:
            self.lifecycle.computation_status(ready=False, reason="worker_unavailable")
            self.capture()
            return None
        dispatch = self.lifecycle.dispatch_cycle()
        if dispatch is None:
            self.capture()
            return None
        self.dispatches.append(dispatch)
        source = dispatch.inputs
        timed = (source.joystick, source.robot_state, *source.history)
        self.event("dispatch", {"snapshot_id": source.snapshot.snapshot_id, "command_seq": dispatch.command_seq,
            "created_at": self.clock.stamp(source.created_ns), "dispatched_at": self.clock.stamp(dispatch.dispatched_ns),
            "expires_at": self.clock.stamp(source.expires_ns), "solve_deadline": self.clock.stamp(dispatch.solve_deadline_ns),
            "inputs": [{"message_id": v.message.meta.message_id, "received_at": self.clock.stamp(v.received_ns),
                        "source_age_ns": source.created_ns - v.sampled_ns} for v in timed]})
        intent = estimate_intent(source.resolved, self.intent_config,
            weights_id=f"weights-{source.snapshot.snapshot_id}", policy=NUMERIC_OFFLINE_V1)
        if intent.disposition != "active":
            self.lifecycle.computation_status(ready=False, reason="intent_" + intent.disposition)
            self.capture()
            return None
        weights = self.envelope("ObjectWeights", intent.weights)
        self.messages[weights.meta.message_id] = weights
        previous = source.previous_applied
        reference = None if previous is None else AppliedCommandReference(previous.result, previous.acceptance,
            previous.source.resolved if previous.source else None)
        # Map the ORIGINAL dispatch deadline once; input/gate callbacks never
        # wait on the Future and result arrival never extends this deadline.
        deadline = time.monotonic_ns() + max(0, dispatch.solve_deadline_ns - self.clock.now)
        future = self.optimizer.solve(source.resolved, intent.weights, intent_config=self.intent_config,
            command_seq=dispatch.command_seq, previous_applied=reference,
            result_id=f"result-{source.snapshot.snapshot_id}", deadline_ns=deadline)
        self.pending = (future, weights, dispatch)
        self.capture()
        return dispatch

    def poll(self, *, deliver=True):
        if self.pending is None or not self.pending[0].done():
            return None
        future, weights, dispatch = self.pending
        self.pending = None
        outcome = future.result()  # Done check above; no worker wait here.
        self.solves.append(outcome)
        result = self.envelope("ControlResult", outcome.result)
        self.messages[result.meta.message_id] = result
        if outcome.result.status == "move":
            self.event("evaluation", {"result_id": outcome.result.result_id, "pose": to_dict(outcome.result.next_tcp_pose)})
        if deliver:
            decision = self.submit(result, weights)
        else:
            decision = None
        return result, weights, decision, dispatch

    def submit(self, result, weights):
        self.event("result_arrival", {"result_id": result.payload.result_id, "received_at": self.clock.stamp()})
        decision = self.lifecycle.submit_result(result, weights=weights)
        self.capture()
        return decision

    def step(self, *, u=None, release=False, feedback=True, deliver=True):
        """Same-time withdrawal, plant, feedback/input, results, watchdogs."""
        self.clock.now += PLANT_TICK_NS
        if release:
            self.input((0., 0.))
        try:
            self.plant.advance()
        except ControllerError:
            self.lifecycle.controller_status(**asdict(self.plant.status))
        self.capture()
        if feedback:
            self.feedback()
        if u is not None and not release:
            self.input(u)
        delivered = self.poll(deliver=deliver)
        self.lifecycle.tick()
        self.capture()
        return delivered

    def wait(self, *, deliver=True, max_ticks=60):
        """Paced simulation pump; local callbacks keep running during a solve."""
        for _ in range(max_ticks):
            time.sleep(PLANT_TICK_NS / 1e9)
            delivered = self.step(u=self.u, deliver=deliver)
            if delivered is not None:
                return delivered
        raise RuntimeError("supervised worker did not retire")

    def cycle(self, u=(1., 0.)):
        self.feedback()
        self.input(u)
        if self.dispatch() is None:
            raise RuntimeError(f"cannot dispatch in {self.lifecycle.state}")
        return self.wait()

    def capture(self):
        # Preserve causal append order across components. Typed events serve
        # common inspection; complete dataclass exports are retained separately.
        for e in self.plant.events[self._seen_plant:]:
            rid = e.target.result.payload.result_id if e.target else None
            kind = "controller_observation" if e.operation in ("advance", "sample") else "controller_request"
            self.event(kind, {"at": self.clock.stamp(e.at_ns), "result_id": rid, "detail": asdict(e)})
        self._seen_plant = len(self.plant.events)
        for decision in self.gate.decisions[self._seen_gate:]:
            if decision.applied:
                message = decision.applied.result
                self.messages[message.meta.message_id] = message
            self.event("acceptance", to_dict(decision.proposal))
            if decision.applied and decision.applied.acceptance != decision.proposal:
                self.event("acceptance", to_dict(decision.applied.acceptance))
        self._seen_gate = len(self.gate.decisions)
        for record in self.lifecycle.records[self._seen_lifecycle:]:
            self.event("lifecycle", {"at": self.clock.stamp(record.at_ns), "result_id": None, "detail": record.to_dict()})
        self._seen_lifecycle = len(self.lifecycle.records)

    def close(self):
        if self.lifecycle.state in (S.ACTIVE, S.READY):
            self.input((0., 0.))
        if self.optimizer is not None:
            self.retirements.append(self.optimizer.close())
            self.optimizer = None
        for future in self.retirements:
            future.result(10)  # Disabled motion; never in a local callback.
        self.capture()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
