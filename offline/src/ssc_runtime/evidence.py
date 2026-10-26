"""Export observed runtime records through the server-owned immutable archive."""

from dataclasses import asdict
from hashlib import sha256
from importlib import metadata
from importlib.resources import files
import json
from pathlib import Path
import re
import subprocess
import sys
from zipfile import ZipFile

from ssc_core import EVALUATOR_LIMITS, NUMERIC_OFFLINE_V1, get_solver_config, to_dict
from ssc_core.archive import ArchiveBuilder, environment
from .scenarios import CORE_REVISION, MAPPING


def json_bytes(value):
    return json.dumps(value, sort_keys=True, allow_nan=False).encode()


def wheel_identity(path, distribution, revision):
    """Check supplied wheel against installed package bytes before labelling evidence."""
    if not re.fullmatch(r"[0-9a-f]{40}", revision):
        raise ValueError("source revision must be a full lowercase Git commit SHA")
    path = Path(path)
    dist = metadata.distribution(distribution)
    namespace = distribution.replace("-", "_")
    with ZipFile(path) as wheel:
        for entry in wheel.namelist():
            if entry.startswith(namespace + "/") and not entry.endswith("/"):
                installed = Path(dist.locate_file(entry))
                if not installed.is_file() or installed.read_bytes() != wheel.read(entry):
                    raise ValueError(f"wheel differs from installed package: {entry}")
        version_path = f"{namespace}-{dist.version}.dist-info/METADATA"
        if version_path not in wheel.namelist():
            raise ValueError(f"wheel version differs from installed {distribution}")
    return {"revision": revision, "package_version": dist.version, "wheel_sha256": sha256(path.read_bytes()).hexdigest()}


def installation(core_wheel, runtime_wheel, runtime_revision):
    if metadata.version("ssc-core") != "0.9.0" or metadata.version("ssc-runtime") != "0.6.0":
        raise ValueError("unsupported installed version pair")
    return {"server": wheel_identity(core_wheel, "ssc-core", CORE_REVISION),
            "ros2": wheel_identity(runtime_wheel, "ssc-runtime", runtime_revision)}


def export_run(loop, path, software):
    loop.capture()
    manifest = {"run_id": "offline-loop-" + loop.fixture.variant, "session_id": loop.session_id,
        "schema_version": "1.0", "function_api_version": "geometry-residual-v1",
        "profile_id": "offline-reference-v1", "evaluator_policy_id": "numeric-offline-v1",
        "solver_config_id": "solver-slsqp-v1", "installation_id": "integration-installation-v1",
        "scenario_id": "integration-scenario-v1", "environment": environment(), "software": software,
        "evidence_scope": "Offline software integration with real bounded solver/evaluator processes and synthetic plant feedback. "
                          "Simulation time uses explicit 20 ms ticks; wall-clock computation budgets are independently enforced. "
                          "No hardware, capture-time transform, physical stopping or research-benefit claim.",
        "replay_tolerances": {"atol": 1e-9, "rtol": 1e-9}}
    builder = ArchiveBuilder(manifest)

    def resource(identifier, value):
        builder.add_resource(identifier, json_bytes(value), media_type="application/json")

    for kind, record in loop.fixture.registered:
        builder.add_record(kind, record)
    for message in loop.messages.values():
        builder.add_message(message)
    for dispatch in loop.dispatches:
        builder.add_record("ControlSnapshot", dispatch.inputs.snapshot)
    for event in loop.events:
        builder.add_event(event["event_id"], event["kind"], event["data"])
    profile = files("ssc_runtime").joinpath("data/offline-reference-v1.md").read_bytes()
    for identifier in ("offline-reference-v1", "synthetic-rotary-v1", "synthetic-host-v1"):
        builder.add_resource(identifier, profile, media_type="text/markdown; charset=utf-8")
    resource("numeric-offline-v1", {"validation": asdict(NUMERIC_OFFLINE_V1), "execution": asdict(EVALUATOR_LIMITS)})
    resource("solver-slsqp-v1", asdict(get_solver_config()))
    resource(loop.intent_config.estimator_config_id, asdict(loop.intent_config))
    resource(MAPPING, {"axes": "synthetic_base XY", "dead_zone": 0, "source": "normalized synthetic input events"})
    observation = loop.fixture.records["SceneObservation"]
    resource(observation["calibration_id"], {"scope": "synthetic metadata; P05 adapter behavior deferred",
        "K": observation["K"], "projection_model": observation["projection_model"], "transform": observation["T_control_camera"]})
    for identifier, (content, media) in loop.fixture.assets.items():
        builder.add_resource(identifier, content, media_type=media)
    resource("integration-installation-v1", {"software": software,
        "freeze": subprocess.check_output([sys.executable, "-m", "pip", "freeze"], text=True),
        "command": "Install the retained pinned wheels; python -I -m ssc_runtime.runner (see CLI arguments).",
        "profile_source_revision": CORE_REVISION, "profile_sha256": sha256(profile).hexdigest()})
    resource("integration-scenario-v1", {"fixture": loop.fixture.variant, "fixture_records": loop.fixture.records,
        "tick_ns": 20_000_000, "provenance": loop.plant.provenance,
        "input_messages": [to_dict(m) for m in loop.messages.values() if m.message_type == "JoystickSample"]})
    resource("snapshot-runtime-evidence-v1", {d.inputs.snapshot.snapshot_id: asdict(d) for d in loop.dispatches})
    resource("gate-runtime-evidence-v1", {"decisions": [asdict(d) for d in loop.gate.decisions],
        "arrivals": [asdict(a) for a in loop.gate.arrivals]})
    resource("solver-runtime-evidence-v1", {"preparations": loop.preparations,
        "solves": [asdict(s) for s in loop.solves], "cancellations": loop.cancellations})
    resource("validation-runtime-evidence-v1", [asdict(r) for r in loop.fixture.validation])
    resource("plant-runtime-evidence-v1", {"provenance": loop.plant.provenance,
        "events": [asdict(e) for e in loop.plant.events]})
    resource("lifecycle-runtime-evidence-v1", [r.to_dict() for r in loop.lifecycle.records])
    resource("callback-timing-evidence-v1", {"scope": "Observed serialized software callback durations, "
        "including local release/fault-stop submissions; no concurrent adapter contention or hardware bound.",
        "samples": loop.callback_timings})
    archive = builder.seal()
    digest = archive.write(path)
    return {"path": str(path), "sha256": digest,
            "payload_sha256": json.loads(archive.to_bytes())["sha256"],
            "software": software, "snapshots": len(loop.dispatches),
            "accepted_moves": sum(d.disposition == "accepted_move" for d in loop.gate.decisions)}
