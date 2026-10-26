"""Public synthetic fixture variants, with fresh identities for changed content."""

import base64
from hashlib import sha256
from importlib.resources import files
import json

from ssc_core import (GeometryRef, HostBindingPolicy, HostNormalization, ProgramRef,
                      TaskRef, UnitReview, feature_definition_digest, program_content_digest)
from .activation import ActivationRequest, ActiveBundleStore

CORE_REVISION = "9ce2d2467f10ca6d82e7672a3158aa6f584ebfde"
MAPPING = "integration-xy-v1"
# A real one-pixel PNG and float32 zero depth. Metadata only, never perception.
PNG = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAIAAACQd1PeAAAADElEQVR4nGNgYGAAAAAEAAH2FzhVAAAAAElFTkSuQmCC")
VARIANTS = ("default", "joystick-only", "positive-approach", "within-tolerance", "computation-error")


class Fixture:
    def __init__(self, variant="default"):
        if variant not in VARIANTS:
            raise ValueError(f"unknown fixture: {variant}")
        self.variant = variant
        self.catalog = json.loads(files("ssc_runtime").joinpath("data/bottle_cup_offline_v1.json").read_bytes())
        self.records = {m["message_type"]: m["payload"] for m in self.catalog["messages"]}
        self.programs = self.catalog["constraint_programs"]
        config, geometry, semantic, observation = (self.records[n] for n in
            ("SessionConfig", "SceneGeometry", "SemanticContext", "SceneObservation"))
        suffix = "-integration-" + variant + "-v1"
        observation["observation_id"] += suffix
        observation["calibration_id"] += suffix
        for kind in ("SceneGeometry", "SemanticContext"):
            self.records[kind]["observation_id"] = observation["observation_id"]
        geometry["geometry_id"] += suffix
        geometry["feature_set_id"] += suffix
        semantic["feature_set_id"] = geometry["feature_set_id"]
        semantic["semantic_id"] += suffix
        observation["rgb_ref"].update(asset_id="integration-rgb-v1", shape=[1, 1, 3])
        observation["depth_ref"].update(asset_id="integration-depth-v1", shape=[1, 1])
        self.assets = {"integration-rgb-v1": (PNG, "image/png"),
                       "integration-depth-v1": (bytes(4), "application/octet-stream")}
        if variant == "joystick-only":
            config["config_id"] = "config-offline-alpha0-v1"
        self.intent_id = "intent-linear-alpha0-v1" if variant == "joystick-only" else "intent-linear-v1"
        if variant in ("positive-approach", "within-tolerance", "computation-error"):
            self.programs = self.programs[:1]
            geometry["objects"] = geometry["objects"][:1]
            geometry["keypoints"] = geometry["keypoints"][:1]
            semantic["object_relevance"] = semantic["object_relevance"][:1]
        if variant in ("positive-approach", "within-tolerance"):
            geometry["keypoints"][0]["position_xyz"] = [.42, 0., .3]
            self.programs[0]["parameters"][0]["value"] = .01 if variant == "positive-approach" else .03
        if variant == "computation-error":
            program = self.programs[0]
            program.update(source_code="def evaluate(p_tcp, R_control_tcp, keypoints, axis_dirs, params):\n    return params[0] / (p_tcp[0] - params[1])\n",
                keypoint_bindings=[], output_unit="dimensionless", parameters=[
                    {"name": "numerator", "value": .01, "unit": "m"},
                    {"name": "pole_x", "value": .405, "unit": "m"}])
        for p in self.programs:
            p["program_id"] += suffix
            p["source_sha256"] = sha256(p["source_code"].encode()).hexdigest()
        semantic["program_refs"] = [{"program_id": p["program_id"], "source_sha256": p["source_sha256"]}
                                    for p in self.programs]
        self.validation = []
        self.registered = []

    def create_store(self, session_id):
        geometry, config, task = (self.records[n] for n in ("SceneGeometry", "SessionConfig", "TaskContext"))
        reviews = tuple(UnitReview("review-" + p["program_id"], program_content_digest(p),
            geometry["scene_id"], geometry["feature_set_id"], feature_definition_digest(geometry),
            p["output_unit"], tuple(v["unit"] for v in p["parameters"]), "offline fixture formula review",
            "Distance minus tolerance in metres; division variant divides two metre quantities.") for p in self.programs)
        policies = tuple(HostBindingPolicy(config["config_id"], p["program_id"],
            HostNormalization(p["output_unit"], 1. if p["output_unit"] == "dimensionless" else .01)) for p in self.programs)
        store = ActiveBundleStore(session_id, unit_reviews=reviews, binding_policies=policies)
        bindings = []
        with store.artifacts() as (registry, validator):
            for kind, value in self.records.items():
                self.registered.append((kind, registry.register_record(kind, value, session_id=session_id)))
            for p in self.programs:
                self.registered.append(("ConstraintProgram", registry.register_record("ConstraintProgram", p, session_id=session_id)))
                report = validator.validate(p, geometry=geometry, config_id=config["config_id"], motion_disabled=True)
                self.validation.append(report)
                if report.receipt.status != "accepted":
                    raise RuntimeError(f"fixture validation failed: {report.receipt.diagnostics}")
                binding = validator.create_binding("host-" + p["program_id"], receipt_id=report.receipt.receipt_id,
                    geometry_ref=GeometryRef(geometry["geometry_id"], geometry["revision"]), config_id=config["config_id"])
                self.registered.extend((("ProgramValidationReceipt", report.receipt), ("ProgramExecutionBinding", binding)))
                bindings.append(binding.binding_id)
        request = ActivationRequest(session_id, config["config_id"], TaskRef(task["task_id"], task["revision"]),
            geometry["scene_id"], geometry["observation_id"], GeometryRef(geometry["geometry_id"], geometry["revision"]),
            geometry["feature_set_id"], self.records["SemanticContext"]["semantic_id"],
            tuple(ProgramRef(p["program_id"], p["source_sha256"]) for p in self.programs), tuple(bindings))
        return store, request
