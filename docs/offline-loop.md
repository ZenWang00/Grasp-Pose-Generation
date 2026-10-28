# Candidate-point offline loop

[Issue #33](https://github.com/ZenWang00/semantic-shared-control/issues/33) changes the
installed pair to `ssc-core==0.11.0` and `ssc-runtime==0.8.0`, schema 2.0,
`candidate-points-v2`, `solver-candidates-v2` and `offline-candidates-v2`.
The exact producer commit is pinned in `offline/pyproject.toml` and CI. Older
schema/program archives remain historical records and are rejected by this reader.

On Linux / Python 3.12, build the producer at that pinned commit and this consumer:

```sh
python3.12 -m pip wheel --no-deps --wheel-dir dist ./shared-core
python3.12 -m pip wheel --no-deps --wheel-dir dist ./offline
python3.12 -m venv .venv-offline
.venv-offline/bin/python -m pip install 'dist/ssc_core-0.11.0-py3-none-any.whl[test]'
.venv-offline/bin/python -m pip install --no-deps dist/ssc_runtime-0.8.0-py3-none-any.whl
.venv-offline/bin/python -m pip check
.venv-offline/bin/python -I -m ssc_runtime.runner \
  --core-wheel dist/ssc_core-0.11.0-py3-none-any.whl \
  --runtime-wheel dist/ssc_runtime-0.8.0-py3-none-any.whl \
  --runtime-revision "$(git rev-parse HEAD)" --output integration-evidence
```

The output directory must be new. The CLI validates wheel bytes against installed
packages, activates context, prepares the supervised solver with motion disabled,
engages after neutral, runs three control cycles, releases and verifies no later
plant motion. It writes `run.json`, checksums/version manifest and fresh-process
`replay.json`. Errors retain `failure.json` and fail the run; they are never
silently retried or labeled successful movement.

Fixtures directly supply grounded TCP candidates in metres. Image/depth assets
are synthetic one-pixel provenance, not perception output. `--candidate-count 1`,
`2` and `16` exercise candidate capacities with stable unique IDs; same-object
alternatives are supported. `--variant higher-target`, `equal-target`, `lower-target`,
`multiple-alternatives` and `joystick-only` exercise height assistance, selection
and alpha=0. `workspace-face` and `excluded-sphere` support boundary tests.
`--cycles` allows 2–20; there is no terminal grasp/hold stage. Candidate distance
provides a soft target while continued joystick input can make corrections.

The loop dispatches an explicit selection and a three-variable fixed-distance
solve. Orientation is copied from measured source state. Simulation advances only
in 20 ms ticks; joint-derived feedback evolves independently and never copies a
command target. Real monotonic deadlines bound the separate solver process.
Input/controller callbacks do not wait for solve futures. Release, faults or epoch
changes cancel outstanding work; old results cannot restart or cancel new motion.

Archives retain context, every candidate selection/predecessor, snapshots, source
messages, local arrival/acceptance, solver problem/configuration, plant events and
callback durations. Replay recomputes selection and fixed candidate distance without
executing generated code or importing a motion runtime. Numeric thresholds and
checksum errors remain visible. This is simulation and replay evidence, not a
real-time guarantee, live VLM evaluation, grounding validation or grasp result.

CI builds pinned wheels, runs both repositories' tests, checks profile/fixture
identity, runs 1/2/16 candidate cases and height/switching scenarios, and joins
current test selectors through `offline/tools/acceptance_matrix.py`. The canonical
profile is copied byte-for-byte from the producer. Old generated-code test names
and old program-count evidence do not establish this version's acceptance.
