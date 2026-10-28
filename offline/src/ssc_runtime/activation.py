"""Atomic, host-owned context activation, independent of motion permission.

The state lock never waits for the artifact lock: cancellation and replacement
remain available while a validator is working. All registry/authority use must
go through artifacts(). Generated data cannot supply initialization tokens.
"""

from contextlib import contextmanager
from dataclasses import dataclass
from threading import RLock
from typing import Literal
from uuid import uuid4

from ssc_core import (
    GeometryRef,
    NUMERIC_CANDIDATES_V2, RegistryError, ResolvedBundle,
    SceneObservation, SessionRegistry, TaskRef, ValidationError,
    parse_record, to_dict,
)


def _text(value, name):
    if type(value) is not str or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")


@dataclass(frozen=True, slots=True)
class HostCompatibility:
    """Trusted installed host identities, not values inferred from a request."""

    control_frame: str = "synthetic_base"
    tcp_frame: str = "synthetic_tcp"
    robot_model_id: str = "synthetic-rotary-v1"
    motion_constraint_set_id: str = "synthetic-host-v1"

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            _text(getattr(self, name), name)


@dataclass(frozen=True, slots=True)
class ActivationRequest:
    """Exact context requested by the local coordinator before preparation."""

    session_id: str
    config_id: str
    task_ref: TaskRef
    scene_id: str
    observation_id: str
    geometry_ref: GeometryRef
    feature_set_id: str
    semantic_id: str
    candidate_set_id: str

    def __post_init__(self):
        for name in ("session_id", "config_id", "scene_id", "observation_id",
                     "feature_set_id", "semantic_id"):
            _text(getattr(self, name), name)
        for name, cls in (("task_ref", TaskRef), ("geometry_ref", GeometryRef)):
            value = getattr(self, name)
            if type(value) is not cls:
                raise TypeError(f"{name} must be {cls.__name__}")
            object.__setattr__(self, name, parse_record(cls, to_dict(value), policy=NUMERIC_CANDIDATES_V2))
        _text(self.candidate_set_id, "candidate_set_id")


@dataclass(frozen=True, slots=True, eq=False)
class InitializationAttempt:
    """Store-issued token. Equal-looking reconstructions are not current tokens."""

    attempt_id: str
    request: ActivationRequest


@dataclass(frozen=True, slots=True, eq=False)
class ActiveBundle:
    """Immutable retained data; eligibility belongs to the store, not this value."""

    activation_id: str
    attempt: InitializationAttempt
    resolved: ResolvedBundle
    observation: SceneObservation
    semantic_observation: SceneObservation


@dataclass(frozen=True, slots=True, eq=False)
class ActivationOutcome:
    status: Literal["prepared", "activated", "rejected", "superseded", "invalidated"]
    attempt_id: str | None
    code: str
    diagnostic: str
    bundle: ActiveBundle | None = None


class ActiveBundleStore:
    """One session's context store, with explicit prepare/publish boundaries.

    begin() and invalidate() immediately withdraw context eligibility. They do
    not stop a controller: the lifecycle owner must withdraw motion authority,
    advance its epoch and request hold/stop before calling them. acquire() is
    snapshot data, never a lease or command permission. Final gates must recheck
    context currency and their independent authority/freshness requirements.
    """

    def __init__(self, session_id: str, *, compatibility: HostCompatibility = HostCompatibility()):
        if type(compatibility) is not HostCompatibility:
            raise TypeError("expected trusted HostCompatibility")
        self._registry = SessionRegistry(session_id, policy=NUMERIC_CANDIDATES_V2)
        self._compatibility = compatibility
        self._artifact_lock = RLock()
        self._state_lock = RLock()
        self._attempt: InitializationAttempt | None = None
        self._preparing: InitializationAttempt | None = None
        self._prepared: ActivationOutcome | None = None
        self._active: ActiveBundle | None = None

    @property
    def session_id(self) -> str:
        return self._registry.session_id

    @contextmanager
    def artifacts(self):
        """Serialize candidate and context registration; retain immutable records only."""
        with self._artifact_lock:
            yield self._registry

    def begin(self, request: ActivationRequest) -> InitializationAttempt:
        if type(request) is not ActivationRequest:
            raise TypeError("expected ActivationRequest")
        if request.session_id != self.session_id:
            raise ValueError("session_mismatch: create a new store for a new session")
        with self._state_lock:
            attempt = InitializationAttempt(str(uuid4()), request)
            self._attempt = attempt
            self._active = self._prepared = self._preparing = None
            return attempt

    def acquire(self) -> ActiveBundle | None:
        with self._state_lock:
            return self._active

    def is_current(self, bundle: ActiveBundle) -> bool:
        with self._state_lock:
            return bundle is not None and bundle is self._active

    @contextmanager
    def command_context(self):
        """Internal gate lease. Acquire coordinator before this; no artifacts.

        Only short local checks/submissions belong here. Adapter callbacks must
        not reenter the runtime. Numerical preparation never runs in this guard.
        """
        with self._state_lock:
            yield self._active

    def invalidate(self, reason: str) -> ActivationOutcome:
        _text(reason, "reason")
        with self._state_lock:
            attempt_id = self._attempt.attempt_id if self._attempt else None
            self._attempt = self._preparing = self._prepared = self._active = None
            return ActivationOutcome("invalidated", attempt_id, "context_invalidated", reason)

    @staticmethod
    def _superseded(attempt):
        return ActivationOutcome("superseded", getattr(attempt, "attempt_id", None),
                                 "superseded_attempt", "attempt is no longer pending in this store")

    def _resolve(self, attempt: InitializationAttempt) -> ActiveBundle:
        request = attempt.request
        with self.artifacts() as registry:
            resolved = registry.resolve_bundle(
                config_id=request.config_id, task_ref=request.task_ref,
                geometry_ref=request.geometry_ref, semantic_id=request.semantic_id,
                candidate_set_id=request.candidate_set_id)
            for name in self._compatibility.__dataclass_fields__:
                if getattr(resolved.configuration, name) != getattr(self._compatibility, name):
                    raise RegistryError("activation", "host_incompatible", f"configuration.{name} differs from installed host")
            for name in ("scene_id", "observation_id", "feature_set_id"):
                if getattr(resolved.geometry, name) != getattr(request, name):
                    raise RegistryError("activation", "context_mismatch", f"geometry.{name} differs from requested context")
            observation = registry.get_record(SceneObservation, request.observation_id,
                                               session_id=self.session_id)
            semantic_observation = registry.get_record(SceneObservation, resolved.semantic.observation_id,
                                                        session_id=self.session_id)
            return ActiveBundle(str(uuid4()), attempt, resolved, observation, semantic_observation)

    def prepare(self, attempt: InitializationAttempt) -> ActivationOutcome:
        with self._state_lock:
            if attempt is not self._attempt or attempt is None:
                return self._superseded(attempt)
            if self._prepared is not None or self._preparing is attempt:
                return ActivationOutcome("rejected", attempt.attempt_id, "attempt_busy",
                                         "preparation already started; publish its result or begin a new attempt")
            self._preparing = attempt
        try:
            bundle = self._resolve(attempt)
            result = ActivationOutcome("prepared", attempt.attempt_id, "compatible_context",
                                       "all candidate and context records verified; publication pending", bundle)
        except (RegistryError, ValidationError) as exc:
            result = ActivationOutcome("rejected", attempt.attempt_id, exc.code, str(exc))
        except Exception:
            # Programming/host failures remain visible to the caller, but must
            # retire this attempt without clearing a newer attempt or bundle.
            with self._state_lock:
                if attempt is self._attempt:
                    self._attempt = self._preparing = self._prepared = self._active = None
            raise
        with self._state_lock:
            if attempt is not self._attempt:
                return self._superseded(attempt)
            self._preparing = None
            if result.status == "rejected":
                self._attempt = self._prepared = self._active = None
            else:
                self._prepared = result
            return result

    def publish(self, prepared: ActivationOutcome) -> ActivationOutcome:
        with self._state_lock:
            if type(prepared) is not ActivationOutcome:
                raise TypeError("expected ActivationOutcome")
            if prepared is not self._prepared or self._attempt is None:
                return ActivationOutcome("superseded", prepared.attempt_id, "unpublishable_result",
                                         "preparation is foreign, failed, consumed or superseded")
            bundle = prepared.bundle
            self._active = bundle
            self._attempt = self._prepared = self._preparing = None
            return ActivationOutcome("activated", prepared.attempt_id, "context_ready",
                                     "context ready; fresh engagement and feedback still required", bundle)
