from copy import deepcopy
import json
from pathlib import Path

import pytest
from ssc_core import GeometryRef, TaskRef
from ssc_runtime import ActivationRequest, ActiveBundleStore


@pytest.fixture
def catalog():
    return json.loads((Path(__file__).parent / "fixtures/bottle_cup_candidates_v2.json").read_text())


@pytest.fixture
def build(catalog):
    def make(*, change=None, session_id="session-offline-v1"):
        data = deepcopy(catalog)
        records = {m["message_type"]: m["payload"] for m in data["messages"]}
        if change:
            change(records, data)
        geometry, config, task = (records[name] for name in ("SceneGeometry", "SessionConfig", "TaskContext"))
        store = ActiveBundleStore(session_id)
        with store.artifacts() as registry:
            for name, record in records.items():
                registry.register_record(name, record, session_id=session_id)
        request = ActivationRequest(session_id, config["config_id"], TaskRef(task["task_id"], task["revision"]),
            geometry["scene_id"], geometry["observation_id"], GeometryRef(geometry["geometry_id"], geometry["revision"]),
            geometry["feature_set_id"], records["SemanticContext"]["semantic_id"],
            records["CandidateSet"]["candidate_set_id"])
        return store, request, records, ()
    return make
