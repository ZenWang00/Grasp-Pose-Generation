"""Grounded TCP candidates for offline simulation; no perception or generated code."""

import base64
from copy import deepcopy
from importlib.resources import files
import json

from ssc_core import GeometryRef, TaskRef
from .activation import ActivationRequest, ActiveBundleStore

CORE_REVISION = "e9f54789b3e62c8bbe01772a3bd96bd581f6fb66"
SOLVER_ID = "solver-candidates-v2"
PROFILE_ID = "offline-candidates-v2"
MAPPING = "integration-xy-v1"
PNG = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAIAAACQd1PeAAAADElEQVR4nGNgYGAAAAAEAAH2FzhVAAAAAElFTkSuQmCC")
VARIANTS = ("default", "joystick-only", "higher-target", "equal-target", "lower-target",
            "multiple-alternatives", "workspace-face", "excluded-sphere")


class Fixture:
    def __init__(self, variant="default", *, candidate_count=None):
        if variant not in VARIANTS:
            raise ValueError(f"unknown fixture: {variant}")
        self.variant = variant
        self.catalog = json.loads(files("ssc_runtime").joinpath("data/bottle_cup_candidates_v2.json").read_bytes())
        self.records = {m["message_type"]: m["payload"] for m in self.catalog["messages"]}
        config, geometry, semantic, observation, candidates = (self.records[n] for n in
            ("SessionConfig", "SceneGeometry", "SemanticContext", "SceneObservation", "CandidateSet"))
        suffix = "-integration-" + variant + "-v2"
        observation["observation_id"] += suffix
        observation["calibration_id"] += suffix
        for kind in ("SceneGeometry", "SemanticContext", "CandidateSet"):
            self.records[kind]["observation_id"] = observation["observation_id"]
        geometry["geometry_id"] += suffix
        geometry["feature_set_id"] += suffix
        semantic["feature_set_id"] = candidates["feature_set_id"] = geometry["feature_set_id"]
        semantic["semantic_id"] += suffix
        candidates["candidate_set_id"] += suffix
        observation["rgb_ref"].update(asset_id="integration-rgb-v1", shape=[1, 1, 3])
        observation["depth_ref"].update(asset_id="integration-depth-v1", shape=[1, 1])
        self.assets = {"integration-rgb-v1": (PNG, "image/png"),
                       "integration-depth-v1": (bytes(4), "application/octet-stream")}
        self.intent_id = "intent-candidates-alpha0-v2" if variant == "joystick-only" else "intent-candidates-v2"
        if variant == "joystick-only":
            config["config_id"] = "config-candidates-alpha0-v2"
        if variant in ("higher-target", "equal-target", "lower-target", "workspace-face", "excluded-sphere"):
            candidates["candidates"] = candidates["candidates"][:1]
            point = candidates["candidates"][0]
            point["position_xyz"] = {
                "higher-target": [.42, 0., .32], "equal-target": [.42, 0., .30],
                "lower-target": [.42, 0., .28], "workspace-face": [.51, 0., .30],
                "excluded-sphere": [.45, .07, .30]}[variant]
        if variant == "multiple-alternatives":
            first = candidates["candidates"][0]
            candidates["candidates"] = [dict(first, candidate_id=first["candidate_id"] + "-high",
                position_xyz=[.42, 0., .32]), dict(first, candidate_id=first["candidate_id"] + "-low",
                position_xyz=[.42, 0., .28])]
        if candidate_count is not None:
            if type(candidate_count) is not int or not 1 <= candidate_count <= 16 or variant != "default":
                raise ValueError("candidate_count requires the default fixture and an integer in 1-16")
            originals = candidates["candidates"]
            candidates["candidates"] = [dict(deepcopy(originals[i % 2]),
                candidate_id=f"candidate-{candidate_count}-{i:02d}") for i in range(candidate_count)]
            candidates["candidate_set_id"] += f"-count-{candidate_count}"
        owners = {point["object_id"] for point in candidates["candidates"]}
        semantic["object_relevance"] = [item for item in semantic["object_relevance"]
                                        if item["object_id"] in owners]
        self.registered = []

    def create_store(self, session_id):
        geometry, config, task = (self.records[n] for n in ("SceneGeometry", "SessionConfig", "TaskContext"))
        store = ActiveBundleStore(session_id)
        with store.artifacts() as registry:
            for kind, value in self.records.items():
                self.registered.append((kind, registry.register_record(kind, value, session_id=session_id)))
        request = ActivationRequest(session_id, config["config_id"], TaskRef(task["task_id"], task["revision"]),
            geometry["scene_id"], geometry["observation_id"], GeometryRef(geometry["geometry_id"], geometry["revision"]),
            geometry["feature_set_id"], self.records["SemanticContext"]["semantic_id"],
            self.records["CandidateSet"]["candidate_set_id"])
        return store, request
