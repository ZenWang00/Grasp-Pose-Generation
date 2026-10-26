"""Run the complete pinned offline loop, release, archive and fresh-process replay."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from .evidence import export_run, installation
from .loop import OfflineLoop
from .lifecycle import LifecycleState as S
from .scenarios import VARIANTS


def run_normal(loop, cycles=3):
    loop.initialize()
    results = []
    for i in range(cycles):
        result, weights, decision, dispatch = loop.cycle((1., 0.))
        if decision.disposition != "accepted_move":
            raise RuntimeError(f"normal case did not move: {decision.proposal.reason}; {loop.solves[-1].diagnostic}")
        results.append((result, weights, decision, dispatch))
        # Independent plant evolution after receipt; never set state to target.
        next_cycle = dispatch.dispatched_ns + 200_000_000
        while loop.clock.now < next_cycle:
            loop.step(u=loop.u)
    loop.step(release=True)
    if loop.lifecycle.state is not S.PAUSED or loop.plant.target is not None:
        raise RuntimeError("release did not withdraw motion")
    stopped = loop.plant.sample().payload.joint_positions_rad
    loop.step()
    if loop.plant.sample().payload.joint_positions_rad != stopped:
        raise RuntimeError("plant advanced after release")
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core-wheel", required=True)
    parser.add_argument("--runtime-wheel", required=True)
    parser.add_argument("--runtime-revision", required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory; never overwrite existing evidence")
    parser.add_argument("--variant", choices=VARIANTS, default="default")
    parser.add_argument("--cycles", type=int, default=3)
    args = parser.parse_args(argv)
    if sys.platform != "linux":
        parser.error("bounded numerical execution requires Linux")
    if not 2 <= args.cycles <= 20:
        parser.error("cycles must be between 2 and 20")
    software = installation(args.core_wheel, args.runtime_wheel, args.runtime_revision)
    args.output.mkdir(parents=True, exist_ok=False)
    with OfflineLoop(args.variant) as loop:
        try:
            run_normal(loop, args.cycles)
        except Exception as exc:
            # Preserve diagnostics and report failure; never silently retry a
            # failed golden run or label a timeout as successful movement.
            (args.output / "failure.json").write_text(json.dumps({"error": str(exc),
                "state": loop.lifecycle.state.value, "solves": [s.code for s in loop.solves]}, indent=2))
            raise
        loop.close()
        evidence = export_run(loop, args.output / "run.json", software)
    replay = subprocess.run([sys.executable, "-I", "-m", "ssc_core.replay", str(args.output / "run.json"),
        "--sha256", evidence["sha256"]], capture_output=True, text=True, timeout=60)
    (args.output / "replay.json").write_text(replay.stdout)
    (args.output / "replay.stderr").write_text(replay.stderr)
    evidence["replay_returncode"] = replay.returncode
    (args.output / "manifest.json").write_text(json.dumps(evidence, indent=2))
    print(json.dumps(evidence, indent=2))
    return 0 if replay.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
