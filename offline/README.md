# Offline coordination package

`ssc-runtime==0.7.0` implements the robot-owned atomic ActiveBundle store,
immutable snapshot/local dispatch coordination, local command acceptance, and
the coordinator lifecycle with local withdrawal, watchdogs and recovery, and
a deterministic synthetic controller/feedback plant for
[issue #1](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/1),
[issue #2](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/2),
[issue #3](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/3),
[issue #4](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/4) and
[issue #5](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/5).
It imports no ROS, drivers, GPU libraries or online services. Installation uses
the exact accepted server revision `4dbf12bcab2d7577fd9e477ab28171b7336bd406`
(`ssc-core==0.10.0`, schema 1.0, geometry-residual-v1). It also provides the complete [offline loop runner](../docs/offline-loop.md)
with asynchronous fusion/optimization, local control, immutable archival and fresh-process replay.

From the repository root, with Python 3.12:

```sh
python3.12 -m venv .venv-offline
.venv-offline/bin/python -m pip install './offline[test]'
.venv-offline/bin/python -m pytest offline/tests
.venv-offline/bin/python -m pip check
```

The install fetches the pinned shared package source. Runtime is offline.
The inherited ROS workspace and requirements are not used. Linux is required
for real bounded numerical validation; macOS tests explicitly stub numerical
success for activation/trust logic, and skip the real-worker integration test.
This does not claim macOS resource isolation or hardware validation.

See [the activation interface](../docs/active-bundle.md) for ownership,
concurrency, failure semantics, evidence mapping and downstream responsibilities.
See [snapshot and dispatch interfaces](../docs/snapshots.md) for local clock
requirements, immutable worker inputs and accepted-command history.
See [command acceptance and synthetic constraints](../docs/command-gate.md)
for the trusted action boundary, ordering, independent live checks and evidence.
See [lifecycle, withdrawal and watchdogs](../docs/lifecycle.md) for engagement,
event-specific recovery, FAULT rearm, session/clock restarts and adapter handoff.

See [the synthetic controller and feedback plant](../docs/synthetic-plant.md)
for explicit clock steps, simulation provenance, failure injection and runner wiring.
