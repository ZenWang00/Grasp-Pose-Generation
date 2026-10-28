# Candidate-point offline coordination

`ssc-runtime==0.8.0` consumes `ssc-core==0.11.0`, schema 2.0 and the
[candidate profile](https://github.com/ZenWang00/semantic-shared-control/blob/main/docs/wiki/Offline-Candidate-Profile.md).
[Issue #33](https://github.com/ZenWang00/semantic-shared-control/issues/33) replaces
VLM-generated functions with grounded TCP target positions. One selected candidate
provides soft three-dimensional position assistance; measured orientation is held.
The joystick still controls XY motion, gates step size and stops motion on release.

This package owns atomic activation, immutable snapshots, explicit selection
history, supervised asynchronous solve dispatch, final local command acceptance,
withdrawal/watchdogs and independent synthetic joint-derived feedback. It imports
no ROS, drivers, GPU libraries or online services. Raw image proposals and real
RGB-D grounding are outside this offline loop.

From the repository root with Python 3.12:

```sh
python3.12 -m venv .venv-offline
.venv-offline/bin/python -m pip install './offline[test]'
.venv-offline/bin/python -m pytest offline/tests
.venv-offline/bin/python -m pip check
```

Installation fetches the exact core revision pinned in `pyproject.toml`; execution
is offline. Linux is required for supervised solver resource enforcement and
full-loop evidence. macOS runs the deterministic component tests; Linux-only
integration tests are explicitly skipped there. These are software simulation
checks, not hardware or grasp-success evidence.

- [Activation](../docs/active-bundle.md): complete candidate context and atomic replacement.
- [Snapshots](../docs/snapshots.md): clocks, frozen input, candidate selection and accepted history.
- [Command gate](../docs/command-gate.md): independent checks and local motion authority.
- [Lifecycle](../docs/lifecycle.md): release, stale input, cancellation and recovery.
- [Synthetic plant](../docs/synthetic-plant.md): joint-derived feedback and failure injection.
- [Complete offline loop](../docs/offline-loop.md): wheel installation, scenarios and replay.
