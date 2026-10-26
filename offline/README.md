# Offline coordination package

`ssc-runtime==0.2.0` implements the robot-owned atomic ActiveBundle store and
immutable snapshot/local dispatch coordination for
[issue #1](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/1) and
[issue #2](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/2).
It imports no ROS, drivers, GPU libraries or online services. Installation uses
the exact accepted server revision `95d618f567f5f057a1b3f1737cc394a105fa9a4e`
(`ssc-core==0.6.0`, schema 1.0, geometry-residual-v1). It does not implement the
full offline loop, command gate, lifecycle or controller.

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
