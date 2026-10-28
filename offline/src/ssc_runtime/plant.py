"""Deterministic synthetic-rotary-v1 controller, never hardware feedback.

The caller serializes events, advances its integer clock, and calls advance()
once per 20 ms tick. No background thread, wall clock, lifecycle callback or
implicit catch-up is used. apply() is a trusted CommandGate boundary.
"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from ssc_core import Message, NUMERIC_CANDIDATES_V2, Pose, parse_message, to_dict

from .constraints import (ANGLE_LIMIT, JOINT_NAMES, SyntheticHostChecker,
                          _configuration, basic_step_constraints, measured_view,
                          proxy_forward, proxy_inverse)
from .gate import AcceptedTarget
from .snapshots import NS, _duration


PLANT_TICK_NS = 20_000_000


class ControllerError(RuntimeError):
    """Failed synthetic submission or step; the runner must report status."""


@dataclass(frozen=True, slots=True)
class ControllerStatus:
    ready: bool
    fault: str | None


@dataclass(frozen=True, slots=True)
class PlantEvent:
    """Local evidence, not a new public message or physical execution claim."""

    sequence: int
    at_ns: int
    operation: str
    outcome: str
    target: AcceptedTarget | None
    joints_before: tuple[float, ...]
    joints_after: tuple[float, ...]
    feedback: Message | None = None
    detail: str | None = None
    provenance: str = "synthetic-rotary-v1:software-simulation"


class SyntheticController:
    """CommandAdapter plus explicitly advanced independent plant and sampler.

    Construct one instance per session/clock domain. Supply the same clock to
    the coordinator. All calls must be serialized by the runner. Immutable
    events, targets and messages can be archived without observing mutable q.
    """

    def __init__(self, configuration, *, session_id: str, clock_ns, clock_id: str,
                 source_id: str = "synthetic-controller", initial_joints=(0.,) * 6):
        self.configuration = _configuration(configuration)
        for name, value in (("session_id", session_id), ("clock_id", clock_id), ("source_id", source_id)):
            if type(value) is not str or not value.strip():
                raise ValueError(f"invalid {name}")
        config = self.configuration
        if (config.control_period_s, config.max_linear_speed_mps,
                config.max_xy_speed_mps, config.max_angular_speed_radps) != (.2, .05, .05, .5):
            raise ValueError("synthetic-rotary-v1 motion settings mismatch")
        self.session_id, self.clock_id, self.source_id = session_id, clock_id, source_id
        self._clock = clock_ns
        self._last_now = -1
        self._state_ns = self._now()
        self._q = tuple(float(v) for v in initial_joints)
        # Validate original values before float conversion could accept bools.
        pose = proxy_forward(initial_joints)
        if any(abs(q) > ANGLE_LIMIT for q in self._q):
            raise ValueError("synthetic_joint_bounds")
        self._velocity = (0.,) * 6
        self._target = None
        self._fault = None
        self._last_command = self._last_epoch = -1
        self._sequence = 0
        self._cached = None
        self._feedback_mode = "normal"
        self._failures = {}
        self._events = []
        initial = self._message(0)
        self._check_pose(pose, initial, (0., 0.), .02)

    @property
    def status(self) -> ControllerStatus:
        return ControllerStatus(self._fault is None, self._fault)

    @property
    def target(self) -> AcceptedTarget | None:
        return self._target

    @property
    def events(self) -> tuple[PlantEvent, ...]:
        return tuple(self._events)

    @property
    def provenance(self) -> dict:
        return {"environment": "software-simulation", "profile_id": "offline-candidates-v2",
                "robot_model_id": "synthetic-rotary-v1", "config_id": self.configuration.config_id,
                "motion_constraint_set_id": self.configuration.motion_constraint_set_id,
                "validation_policy_id": self.configuration.validation_policy_id,
                "session_id": self.session_id, "clock_id": self.clock_id,
                "source_id": self.source_id, "tick_ns": PLANT_TICK_NS,
                "initial_joints_rad": self._events[0].joints_before if self._events else self._q,
                "evidence_level": "synthetic feedback only; no hardware provenance or stopping guarantee"}

    def _now(self):
        now = self._clock()
        if type(now) is not int or now < 0 or now < self._last_now:
            raise ControllerError("invalid_or_regressing_clock")
        self._last_now = now
        return now

    def _stamp(self, now):
        return {"clock_id": self.clock_id, "sec": now // NS, "nanosec": now % NS}

    def _log(self, operation, outcome="ok", *, target=None, before=None, feedback=None, detail=None):
        self._events.append(PlantEvent(len(self._events) + 1, self._now(), operation, outcome,
            target, self._q if before is None else before, self._q, feedback, detail))

    def _message(self, sequence):
        stamp = self._stamp(self._state_ns)
        return parse_message({"message_type": "RobotState", "meta": {
            "schema_version": "2.0", "session_id": self.session_id,
            "message_id": f"{self.session_id}:{self.source_id}:state:{sequence}",
            "source_id": self.source_id, "sequence": sequence, "produced_at": stamp},
            "payload": {"sampled_at": stamp, "frame_id": self.configuration.control_frame,
                "tcp_frame": self.configuration.tcp_frame, "tcp_pose": to_dict(proxy_forward(self._q)),
                "joint_names": list(JOINT_NAMES), "joint_positions_rad": list(self._q),
                "joint_velocities_radps": list(self._velocity), "state_source": "fk_from_measured_joints"}},
            policy=NUMERIC_CANDIDATES_V2)

    def _state(self, message):
        message = parse_message(to_dict(message), policy=NUMERIC_CANDIDATES_V2)
        if message.message_type != "RobotState" or message.meta.session_id != self.session_id:
            raise ValueError("feedback_identity")
        sample = message.payload.sampled_at
        age = self._now() - (sample.sec * NS + sample.nanosec)
        if (sample.clock_id != self.clock_id or age < 0 or
                age > _duration(self.configuration.robot_state_timeout_s)):
            raise ValueError("feedback_not_fresh")
        self._check_pose(message.payload.tcp_pose, message, (0., 0.), .02)
        return message

    def _check_pose(self, pose, state, u_xy, dt):
        kwargs = dict(candidate_pose=pose, measured_state=measured_view(state.payload),
                      u_xy=u_xy, dt_s=dt, configuration=self.configuration)
        for report in (SyntheticHostChecker()(**kwargs), basic_step_constraints(**kwargs)):
            if not report.feasible:
                raise ValueError("plant_constraint: " + ", ".join(report.violation_reasons))
        if any(abs(q) > ANGLE_LIMIT for q in proxy_inverse(pose)):
            raise ValueError("synthetic_joint_bounds")

    def fail_next(self, operation: str):
        """Inject one failed submission/step, with no implicit recovery."""
        if operation not in ("apply", "cancel", "hold", "stop", "advance"):
            raise ValueError("unknown_failure_operation")
        self._failures[operation] = self._failures.get(operation, 0) + 1
        self._log("inject_failure", detail=operation)

    def _attempt(self, operation, *, target=None):
        if self._failures.get(operation, 0):
            self._failures[operation] -= 1
            self._fault = f"injected_{operation}_failure"
            self._log(operation, "failed", target=target, detail=self._fault)
            raise ControllerError(self._fault)

    def inject_controller_fault(self, reason="injected_controller_fault"):
        if type(reason) is not str or not reason.strip():
            raise ValueError("fault reason required")
        self._fault = reason
        self._log("controller_fault", "failed", target=self._target, detail=reason)

    def clear_fault(self):
        """Simulated repair only. The lifecycle still requires explicit rearm."""
        if self._target is not None:
            raise ControllerError("cancel_or_stop_before_fault_clearance")
        self._fault = None
        self._log("clear_fault")

    def set_feedback_mode(self, mode: str):
        if mode not in ("normal", "missing", "stale"):
            raise ValueError("unknown_feedback_mode")
        self._feedback_mode = mode
        self._log("feedback_mode", detail=mode)

    def apply(self, target: AcceptedTarget) -> None:
        """Install a newly gate-accepted target without evolving or sampling q."""
        now = self._now()
        try:
            if self._fault is not None:
                raise ValueError("controller_fault_latched")
            if type(target) is not AcceptedTarget:
                raise ValueError("expected_accepted_target")
            message = parse_message(to_dict(target.result), policy=NUMERIC_CANDIDATES_V2)
            result = message.payload
            if message.message_type != "ControlResult" or message.meta.session_id != self.session_id:
                raise ValueError("target_identity")
            if (result.status != "move" or result.command_seq <= self._last_command or
                    result.control_epoch < self._last_epoch):
                raise ValueError("target_not_new_move")
            if (type(target.accepted_ns) is not int or type(target.expires_ns) is not int or
                    target.accepted_ns != now or result.horizon_s != self.configuration.control_period_s or
                    target.expires_ns != now + _duration(result.horizon_s)):
                raise ValueError("target_timing")
            u = tuple(target.u_xy)
            if (len(u) != 2 or any(type(v) not in (int, float) or not math.isfinite(v) for v in u)
                    or not 0 < math.hypot(*u) <= 1):
                raise ValueError("target_input")
            state = self._state(target.measured_state)
            self._check_pose(result.next_tcp_pose, state, u, result.horizon_s)
            target = AcceptedTarget(message, now, target.expires_ns, u, state)
        except Exception as exc:
            self._log("apply", "rejected", detail=str(exc))
            raise ControllerError(str(exc)) from exc
        self._attempt("apply", target=target)
        replaced = self._target
        self._target = target
        self._last_command, self._last_epoch = result.command_seq, result.control_epoch
        self._log("apply", target=target, detail=None if replaced is None else
                  "replaces:" + replaced.result.payload.result_id)

    def cancel(self) -> None:
        target = self._target
        self._attempt("cancel", target=target)
        self._target = None
        self._log("cancel", target=target)

    def hold(self, measured_state: Message) -> None:
        try:
            state = self._state(measured_state)
        except Exception as exc:
            self._log("hold", "rejected", detail=str(exc))
            raise ControllerError(str(exc)) from exc
        target = self._target
        self._attempt("hold", target=target)
        self._target = None
        self._velocity = (0.,) * 6
        self._log("hold", target=target, feedback=state)

    def stop(self) -> None:
        target = self._target
        self._attempt("stop", target=target)
        self._target = None
        self._velocity = (0.,) * 6
        self._log("stop", target=target)

    def advance(self) -> None:
        """Commit one 20 ms tick, after the runner's same-time withdrawals.

        Reject skipped/repeated ticks rather than retroactively applying a new
        target over unprocessed time. At expiry equality no motion is applied.
        A failed step consumes the tick but commits no joint change.
        """
        now = self._now()
        if now != self._state_ns + PLANT_TICK_NS:
            raise ControllerError("expected_one_20ms_tick")
        target, before = self._target, self._q
        state = self._message(self._sequence)
        self._state_ns = now
        self._velocity = (0.,) * 6
        if target is not None and now >= target.expires_ns:
            self._target = None
            self._log("expire", target=target)
            target = None
        self._attempt("advance", target=target)
        if target is None or self._fault is not None:
            self._log("advance", "idle", target=target, before=before)
            return
        try:
            dt = PLANT_TICK_NS / NS
            current = state.payload.tcp_pose
            desired = target.result.payload.next_tcp_pose
            rotation = Rotation.from_quat(current.quaternion_xyzw)
            dp = np.asarray(desired.position_xyz) - current.position_xyz
            dtheta = (Rotation.from_quat(desired.quaternion_xyzw) * rotation.inv()).as_rotvec()
            magnitude = math.hypot(*target.u_xy)
            limits = [.5]
            for delta, speed in ((dp, self.configuration.max_linear_speed_mps),
                                 (dtheta, self.configuration.max_angular_speed_radps)):
                distance = float(np.linalg.norm(delta))
                if distance:
                    limits.append(speed * magnitude * dt / distance)
            gamma = min(limits)
            pose = Pose(tuple((np.asarray(current.position_xyz) + gamma * dp).tolist()),
                        tuple((Rotation.from_rotvec(gamma * dtheta) * rotation).as_quat().tolist()))
            self._check_pose(pose, state, target.u_xy, dt)
            joints = proxy_inverse(pose)
            self._velocity = tuple((new - old) / dt for new, old in zip(joints, before))
            self._q = joints
        except Exception as exc:
            self._fault = "plant_step_failed: " + str(exc)
            self._log("advance", "failed", target=target, before=before, detail=self._fault)
            raise ControllerError(self._fault) from exc
        self._log("advance", target=target, before=before)

    def sample(self) -> Message | None:
        """Sample committed q; missing/stale modes never renew old identities."""
        self._now()
        if self._feedback_mode == "missing":
            self._log("sample", "missing")
            return None
        if self._feedback_mode == "stale":
            self._log("sample", "stale", feedback=self._cached)
            return self._cached
        cached_time = None if self._cached is None else self._cached.payload.sampled_at
        if cached_time is None or cached_time.sec * NS + cached_time.nanosec != self._state_ns:
            self._sequence += 1
            self._cached = self._message(self._sequence)
        self._log("sample", feedback=self._cached)
        return self._cached
