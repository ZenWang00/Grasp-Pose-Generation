from concurrent.futures import ThreadPoolExecutor
from dataclasses import FrozenInstanceError, replace
import json
import sys
from threading import Barrier, Event

import pytest

from ssc_core import GeometryRef, ProgramRef, TaskRef, to_dict
from ssc_runtime import ActivationOutcome, ActiveBundleStore


def activate(store, request):
    attempt = store.begin(request)
    prepared = store.prepare(attempt)
    assert prepared.status == "prepared", prepared
    assert store.acquire() is None
    outcome = store.publish(prepared)
    assert outcome.status == "activated"
    assert store.acquire() is outcome.bundle
    return outcome.bundle


def register(store, name, record):
    with store.artifacts() as (registry, _):
        return registry.register_record(name, record, session_id=store.session_id)


def test_complete_bundle_and_immutable_retention(build):
    store, request, records, reports = build()
    assert store.acquire() is None
    old = activate(store, request)
    context = old.resolved
    assert context.session_id == request.session_id
    assert context.configuration.config_id == request.config_id
    assert context.task.task_id == request.task_ref.task_id
    assert context.task.revision == request.task_ref.revision
    assert context.geometry.geometry_id == request.geometry_ref.geometry_id
    assert context.geometry.revision == request.geometry_ref.revision
    assert context.geometry.feature_set_id == request.feature_set_id
    assert context.semantic.semantic_id == request.semantic_id
    assert old.observation.observation_id == request.observation_id
    assert old.semantic_observation.observation_id == context.semantic.observation_id
    assert tuple(r.receipt for r in reports) == context.receipts
    assert tuple(p.program.source_sha256 for p in context.programs) == tuple(r.source_sha256 for r in request.program_refs)
    assert all(p.program.function_api_version == "geometry-residual-v1" for p in context.programs)
    assert all(b.normalization_scale == .01 and b.weight == 1 for b in context.execution_bindings)
    with pytest.raises(FrozenInstanceError):
        context.geometry.revision = 99
    with pytest.raises(TypeError):
        context.geometry.keypoints[0].position_xyz[0] = 99
    before = to_dict(context.geometry)
    records["SceneGeometry"]["keypoints"][0]["position_xyz"][0] = 99
    new = activate(store, request)
    assert old is not new and old.activation_id != new.activation_id
    assert to_dict(old.resolved.geometry) == before
    assert not store.is_current(old) and store.is_current(new)


@pytest.mark.parametrize("field,value,code", [
    ("config_id", "absent", "missing_reference"),
    ("task_ref", TaskRef("absent", 1), "missing_reference"),
    ("task_ref", TaskRef("task-approach-offline-v1", 99), "missing_reference"),
    ("geometry_ref", GeometryRef("absent", 1), "missing_reference"),
    ("semantic_id", "absent", "missing_reference"),
    ("scene_id", "foreign-scene", "context_mismatch"),
    ("observation_id", "foreign-observation", "context_mismatch"),
    ("feature_set_id", "foreign-features", "context_mismatch"),
    ("program_refs", (), "program_coverage"),
    ("execution_binding_refs", (), "binding_coverage"),
    ("execution_binding_refs", ("missing-binding",), "missing_reference"),
])
def test_rejection_has_diagnostic_and_no_fallback(build, field, value, code):
    store, request, _, _ = build()
    old = activate(store, request)
    attempt = store.begin(replace(request, **{field: value}))
    assert store.acquire() is None and not store.is_current(old)
    failed = store.prepare(attempt)
    assert failed.status == "rejected" and failed.code == code
    assert failed.diagnostic and failed.bundle is None
    assert store.publish(failed).status == "superseded"
    assert store.acquire() is None


@pytest.mark.parametrize("field", ["robot_model_id", "tcp_frame", "motion_constraint_set_id"])
def test_host_compatibility_is_not_inferred_from_candidate_config(build, field):
    store, request, _, _ = build(change=lambda records, _: records["SessionConfig"].update({field: "other"}))
    failed = store.prepare(store.begin(request))
    assert failed.code == "host_incompatible" and field in failed.diagnostic


def test_control_frame_and_observation_mismatch(build):
    store, request, records, _ = build()
    geometry = records["SceneGeometry"]
    geometry.update(geometry_id="wrong-frame", revision=2, frame_id="foreign")
    register(store, "SceneGeometry", geometry)
    failed = store.prepare(store.begin(replace(request, geometry_ref=GeometryRef("wrong-frame", 2))))
    assert failed.code == "incompatible_reference" and "frame_id" in failed.diagnostic


@pytest.mark.parametrize("field", ["task_revision", "scene_id", "feature_set_id", "observation_id"])
def test_incompatible_semantic_context(build, field):
    store, request, records, _ = build()
    semantic = records["SemanticContext"]
    semantic["semantic_id"] = "changed-semantics"
    semantic[field] = 2 if field == "task_revision" else "foreign"
    register(store, "SemanticContext", semantic)
    failed = store.prepare(store.begin(replace(request, semantic_id=semantic["semantic_id"])))
    assert failed.status == "rejected" and failed.code in ("incompatible_reference", "missing_reference")


@pytest.mark.parametrize("field,value", [("object_id", "missing-object"),
    ("keypoint_bindings", ["missing-keypoint"]), ("axis_bindings", ["missing-axis"])])
def test_missing_program_feature_identity(build, field, value):
    store, request, records, _ = build()
    with store.artifacts() as (registry, _):
        program = to_dict(registry.get_record("ConstraintProgram", request.program_refs[0].program_id,
                                              session_id=store.session_id))
    program.update(program_id="missing-feature-program", **{field: value})
    register(store, "ConstraintProgram", program)
    semantic = records["SemanticContext"]
    semantic["semantic_id"] = "missing-feature-semantics"
    semantic["program_refs"][0]["program_id"] = program["program_id"]
    register(store, "SemanticContext", semantic)
    requested = replace(request, semantic_id=semantic["semantic_id"], program_refs=(
        ProgramRef(program["program_id"], program["source_sha256"]), *request.program_refs[1:]))
    failed = store.prepare(store.begin(requested))
    assert failed.code == "missing_reference" and store.acquire() is None


def test_semantic_capture_can_precede_geometry_capture_in_same_scene(build):
    store, request, records, _ = build()
    observation = records["SceneObservation"]
    observation["observation_id"] = "later-geometry-capture"
    register(store, "SceneObservation", observation)
    geometry = records["SceneGeometry"]
    geometry.update(geometry_id="later-capture-geometry", observation_id=observation["observation_id"])
    register(store, "SceneGeometry", geometry)
    active = activate(store, replace(request, observation_id=observation["observation_id"],
                                      geometry_ref=GeometryRef(geometry["geometry_id"], geometry["revision"])))
    assert active.observation.observation_id != active.semantic_observation.observation_id
    assert active.observation.scene_id == active.semantic_observation.scene_id


@pytest.mark.parametrize("change", ["imported", "rejected", "hash", "feature", "policy", "binding_config"])
def test_receipt_and_binding_boundaries(build, change):
    store, request, _, reports = build()
    report = reports[0]
    with store.artifacts() as (registry, _):
        binding = to_dict(registry.get_record("ProgramExecutionBinding", request.execution_binding_refs[0],
                                             session_id=store.session_id))
    binding["binding_id"] = "changed-binding"
    if change == "binding_config":
        binding["config_id"] = "other-config"
    else:
        receipt = to_dict(report.receipt)
        receipt["receipt_id"] = "imported-receipt"
        if change == "rejected":
            receipt.update(status="rejected", diagnostics=["test rejection"])
        if change == "hash": receipt["source_sha256"] = "0" * 64
        if change == "feature": receipt["feature_set_id"] = "foreign"
        if change == "policy": receipt["validator_policy_version"] = "foreign"
        register(store, "ProgramValidationReceipt", receipt)
        binding["validation_receipt_id"] = receipt["receipt_id"]
    register(store, "ProgramExecutionBinding", binding)
    request = replace(request, execution_binding_refs=(binding["binding_id"], *request.execution_binding_refs[1:]))
    failed = store.prepare(store.begin(request))
    assert failed.status == "rejected"
    assert failed.code == ("untrusted_receipt" if change == "imported" else "incompatible_reference")
    assert store.acquire() is None


def test_wrong_host_normalization_and_missing_binding(build):
    store, request, records, _ = build(issue=False)
    with store.artifacts() as (registry, validator):
        for index, ref in enumerate(request.program_refs):
            p = registry.get_record("ConstraintProgram", ref.program_id, session_id=store.session_id)
            report = validator.validate(p, geometry=records["SceneGeometry"], config_id=request.config_id,
                                        motion_disabled=True)
            register(store, "ProgramExecutionBinding", dict(binding_id=f"binding-{index}", program_id=p.program_id,
                config_id=request.config_id, normalization_scale=.02, weight=1,
                validation_receipt_id=report.receipt.receipt_id))
    request = replace(request, execution_binding_refs=("binding-0", "binding-1"))
    failed = store.prepare(store.begin(request))
    assert failed.code == "normalization_mismatch" and store.acquire() is None


def test_new_store_cannot_restore_archived_trust(build):
    store, request, _, _ = build()
    active = activate(store, request)
    restored = ActiveBundleStore(store.session_id, unit_reviews=(), binding_policies=())
    r = active.resolved
    for name, values in (
        ("SessionConfig", (r.configuration,)), ("TaskContext", (r.task,)),
        ("SceneObservation", (active.observation,)), ("SceneGeometry", (r.geometry,)),
        ("SemanticContext", (r.semantic,)), ("ConstraintProgram", tuple(p.program for p in r.programs)),
        ("ProgramValidationReceipt", r.receipts), ("ProgramExecutionBinding", r.execution_bindings)):
        for value in values: register(restored, name, value)
    failed = restored.prepare(restored.begin(request))
    assert failed.code == "untrusted_receipt"
    assert restored.prepare(active.attempt).status == "superseded"


def test_coordinate_revision_requires_paused_replacement(build):
    store, request, records, _ = build()
    old = activate(store, request)
    geometry = records["SceneGeometry"]
    geometry.update(geometry_id="geometry-revision-2", revision=2)
    geometry["keypoints"][0]["position_xyz"][0] += .01
    register(store, "SceneGeometry", geometry)
    # Registration alone cannot replace or mutate an active reference.
    assert store.acquire() is old
    new = activate(store, replace(request, geometry_ref=GeometryRef(geometry["geometry_id"], 2)))
    assert new.resolved.geometry.revision == 2 and old.resolved.geometry.revision == 1
    assert new.resolved.receipts == old.resolved.receipts
    assert not store.is_current(old)


@pytest.mark.parametrize("replacement", ["same", "task", "scene", "config", "invalidate"])
def test_late_prepared_result_cannot_restore_old_context(build, replacement):
    store, request, _, _ = build()
    first = store.prepare(store.begin(request))
    if replacement == "invalidate":
        result = store.invalidate("scene moved")
        assert result.status == "invalidated"
        new = None
    elif replacement == "same":
        new = activate(store, request)
    else:
        changes = {"task": {"task_ref": TaskRef("next-task", 1)},
                   "scene": {"scene_id": "next-scene"}, "config": {"config_id": "next-config"}}
        pending = store.begin(replace(request, **changes[replacement]))
        assert store.prepare(pending).status == "rejected"
        new = None
    assert store.publish(first).status == "superseded"
    assert store.acquire() is new


def test_concurrent_completion_and_atomic_readers(build, monkeypatch, tmp_path):
    store, request, _, _ = build()
    original = activate(store, request)
    entered, finish, published = Event(), Event(), Event()
    before_publish = Barrier(4)
    resolve = store._resolve
    slow = store.begin(request)
    def delayed(attempt):
        bundle = resolve(attempt)
        if attempt is slow:
            entered.set()
            assert finish.wait(5)
        return bundle
    monkeypatch.setattr(store, "_resolve", delayed)
    trace = [{"stage": "initial", "activation": original.activation_id}]
    with ThreadPoolExecutor(max_workers=5) as pool:
        future = pool.submit(store.prepare, slow)
        try:
            assert entered.wait(5)
            assert store.acquire() is None
            current = store.begin(request)
            prepared = store.prepare(current)
            def read_many():
                assert store.acquire() is None
                seen = [None]
                before_publish.wait(timeout=5)
                assert published.wait(5)
                assert store.acquire() is prepared.bundle
                for _ in range(100):
                    value = store.acquire()
                    assert value is None or value is prepared.bundle
                    if value:
                        assert len(value.resolved.programs) == len(value.resolved.receipts) == 2
                        assert value.resolved.semantic.task_revision == value.resolved.task.revision
                    seen.append(None if value is None else value.activation_id)
                return seen
            readers = [pool.submit(read_many) for _ in range(3)]
            before_publish.wait(timeout=5)
            assert store.publish(prepared).status == "activated"
            published.set()
            for reader in readers:
                trace.append({"stage": "concurrent_read", "values": reader.result(timeout=5)})
        finally:
            finish.set()
            published.set()
        late = future.result(timeout=5)
    assert late.status == "superseded" and late.bundle is None
    assert store.acquire() is prepared.bundle
    assert original.resolved == prepared.bundle.resolved
    trace.append({"stage": "late_completion", "status": late.status,
                  "current": store.acquire().activation_id})
    (tmp_path / "concurrency-trace.json").write_text(json.dumps(trace, indent=2))


def test_invalidate_does_not_wait_for_artifact_validation(build):
    store, request, _, _ = build()
    attempt = store.begin(request)
    with ThreadPoolExecutor() as pool:
        with store.artifacts():
            pending = pool.submit(store.prepare, attempt)
            # Same thread holds artifacts while another performs cancellation:
            # completion proves invalidation does not need the artifact lock.
            invalidated = pool.submit(store.invalidate, "changed task").result(timeout=5)
            assert invalidated.status == "invalidated" and store.acquire() is None
        assert pending.result(timeout=5).status == "superseded"


def test_tokens_duplicate_publication_and_foreign_results(build):
    store, request, _, _ = build()
    attempt = store.begin(request)
    assert store.prepare(replace(attempt)).status == "superseded"
    prepared = store.prepare(attempt)
    assert store.prepare(attempt).code == "attempt_busy"
    assert store.publish(replace(prepared)).status == "superseded"
    assert store.acquire() is None
    assert store.publish(prepared).status == "activated"
    assert store.publish(prepared).status == "superseded"
    assert store.acquire() is prepared.bundle
    with pytest.raises(ValueError, match="session_mismatch"):
        store.begin(replace(request, session_id="other"))


def test_unexpected_host_error_is_visible_and_retires_attempt(build, monkeypatch):
    store, request, _, _ = build()
    attempt = store.begin(request)
    def broken(_): raise RuntimeError("host broke")
    monkeypatch.setattr(store, "_resolve", broken)
    with pytest.raises(RuntimeError, match="host broke"):
        store.prepare(attempt)
    assert store.acquire() is None
    assert store.prepare(attempt).status == "superseded"


@pytest.mark.skipif(sys.platform != "linux", reason="real receipt workers require Linux resource enforcement")
def test_real_worker_receipts_to_complete_activation(build):
    store, request, _, reports = build(stub=False)
    assert len(reports) == 2
    assert all(r.numerical.accepted and r.numerical.cleanup.reaped for r in reports)
    assert all(len(r.numerical.probes) == 50 for r in reports)
    active = activate(store, request)
    assert active.resolved.receipts == tuple(r.receipt for r in reports)
