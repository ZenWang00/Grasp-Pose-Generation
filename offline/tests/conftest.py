from copy import deepcopy
import json
from pathlib import Path

import pytest

from ssc_core import (
    GeometryRef, HostBindingPolicy, HostNormalization, ProgramRef, TaskRef,
    UnitReview, feature_definition_digest, program_content_digest,
)
from ssc_core.program_validation import NumericalValidation
import ssc_core.receipts as receipts
from ssc_runtime import ActivationRequest, ActiveBundleStore


@pytest.fixture
def catalog():
    return json.loads((Path(__file__).parent / "fixtures/bottle_cup_offline_v1.json").read_text())


@pytest.fixture
def build(catalog, monkeypatch):
    def make(*, stub=True, change=None, issue=True):
        data = deepcopy(catalog)
        records = {m["message_type"]: m["payload"] for m in data["messages"]}
        if change:
            change(records, data)
        if stub:
            # Only numerical success is stubbed; registry and host trust are real.
            # Linux end-to-end coverage below uses the actual bounded workers.
            monkeypatch.setattr(receipts, "probe_program", lambda *a, **kw:
                                NumericalValidation(True, (), None, (), None))
        geometry = records["SceneGeometry"]
        config = records["SessionConfig"]
        programs = data["constraint_programs"]
        reviews = tuple(UnitReview(
            "review-" + p["program_id"], program_content_digest(p), geometry["scene_id"],
            geometry["feature_set_id"], feature_definition_digest(geometry), "m", ("m",),
            "offline fixture author", "metre displacement norm minus metre tolerance at the bound approach point")
            for p in programs)
        policies = tuple(HostBindingPolicy(config["config_id"], p["program_id"],
                                          HostNormalization("m", .01)) for p in programs)
        store = ActiveBundleStore("session-offline-v1", unit_reviews=reviews, binding_policies=policies)
        reports, bindings = [], []
        with store.artifacts() as (registry, validator):
            for name in ("SessionConfig", "TaskContext", "SceneObservation", "SceneGeometry", "SemanticContext"):
                registry.register_record(name, records[name], session_id=store.session_id)
            for p in programs:
                registry.register_record("ConstraintProgram", p, session_id=store.session_id)
                if issue:
                    report = validator.validate(p, geometry=geometry, config_id=config["config_id"],
                                                motion_disabled=True)
                    assert report.receipt.status == "accepted", report.receipt.diagnostics
                    reports.append(report)
                    bindings.append(validator.create_binding("host-" + p["program_id"],
                        receipt_id=report.receipt.receipt_id,
                        geometry_ref=GeometryRef(geometry["geometry_id"], geometry["revision"]),
                        config_id=config["config_id"]).binding_id)
        task = records["TaskContext"]
        request = ActivationRequest(store.session_id, config["config_id"],
            TaskRef(task["task_id"], task["revision"]), geometry["scene_id"], geometry["observation_id"],
            GeometryRef(geometry["geometry_id"], geometry["revision"]), geometry["feature_set_id"],
            records["SemanticContext"]["semantic_id"],
            tuple(ProgramRef(p["program_id"], p["source_sha256"]) for p in programs), tuple(bindings))
        return store, request, records, reports
    return make
