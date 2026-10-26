# Complete phase-one offline loop

P1-16 / [issue #6](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/6)
connects `ssc-runtime==0.6.0` to accepted `ssc-core==0.9.0` at
`9ce2d2467f10ca6d82e7672a3158aa6f584ebfde`. Schema 1.0,
geometry-residual-v1 and all reference profile identities stay unchanged.
This is software simulation and numerical replay, not hardware or research evidence.

## Reproduce the installed version pair

Use Linux and Python 3.12. Install/build dependencies once; execution uses no
network, ROS graph, drivers, model downloads or service credentials.
From a clean checkout of this repository at the revision to test:

```sh
git clone https://github.com/ZenWang00/semantic-shared-control.git shared-core
git -C shared-core checkout 9ce2d2467f10ca6d82e7672a3158aa6f584ebfde
python3.12 -m pip wheel --no-deps --wheel-dir dist ./shared-core
python3.12 -m pip wheel --no-deps --wheel-dir dist ./offline
python3.12 -m venv .venv-offline
.venv-offline/bin/python -m pip install 'dist/ssc_core-0.9.0-py3-none-any.whl[test]'
.venv-offline/bin/python -m pip install --no-deps dist/ssc_runtime-0.6.0-py3-none-any.whl
.venv-offline/bin/python -m pip check
```

One command performs validation/activation, neutral then fresh engagement,
three asynchronous fusion/optimization cycles at the 0.2 s control period, local gate
acceptance, independent plant evolution, release, immutable archival and
numerical replay in a fresh process:

```sh
.venv-offline/bin/python -I -m ssc_runtime.runner \
  --core-wheel dist/ssc_core-0.9.0-py3-none-any.whl \
  --runtime-wheel dist/ssc_runtime-0.6.0-py3-none-any.whl \
  --runtime-revision "$(git rev-parse HEAD)" \
  --output integration-evidence
```

The output directory must be new. `run.json` is the self-contained archive;
`manifest.json` retains its independent file checksum, exact Git version pair,
wheel hashes and replay exit status; `replay.json` contains independently
recomputed residuals/penalties and environment differences. The CLI verifies
supplied wheel package bytes against installed files. The caller supplies the
runtime source revision associated with that wheel; CI builds it from the
recorded clean checkout. A failed normal run is an error with retained
`failure.json`, never silently retried or counted as a successful move.

`--variant joystick-only`, `positive-approach` and `within-tolerance` select
named reference inputs; `--cycles` permits 2–20 cycles. The division-error
fixture is an integration-test input and intentionally fails the normal-move
CLI criterion. Initial image/depth bytes are explicit one-pixel synthetic
metadata resources, not perception output. Changed fixture content has fresh
artifact/feature/program IDs. The packaged profile is a byte-for-byte frozen
evidence resource from the pinned server revision; the server Wiki remains
the sole authoring source.

## Runtime boundaries and clocks

`OfflineLoop` owns the serialized `LifecycleCoordinator`, `ActiveBundleStore`,
`SnapshotCoordinator`, `CommandGate` and `SyntheticController`. It injects the
robot-owned `SyntheticHostChecker` into the shared `SupervisedOptimizer`.
Activation uses real host validation receipts. Weights come from the shared
estimator; previous smoothing inputs come exclusively from the gate's accepted
command record, including local holds. Proposed targets never update feedback.

The test clock begins at 10 s and advances explicitly by 20 ms. The simulation
pump waits 20 ms between ticks while polling an asynchronous solve. This is a
controlled simulation schedule, not a measurement of real-time control rate.
The solver independently enforces the unchanged 150 ms real monotonic budget
and 10 ms per evaluator call. Dispatch maps its original local deadline to a
real monotonic deadline once. Timestamps on returned messages never extend it.

Each tick handles same-time release first, then plant advancement, sampled
feedback/current input, eligible results, and local watchdogs. No input,
release or gate callback waits on a worker Future. Preparation and teardown
wait only while motion is disabled. Release requests nonblocking retirement
of an unfinished worker; late results still pass through the local gate and
cannot cancel a newer target. Reusing a retired worker requires preparation
while disabled. Ordinary release reuses valid context; context invalidation
requires activation again; FAULT requires healthy status, explicit rearm,
initialization and fresh engagement. One `OfflineLoop` represents one session;
create another instance/archive for session or clock restart. Each instance uses
a fresh session UUID by default. Same-time archive completion records retain
each component's append order, grouped plant, gate, lifecycle after each host
operation; they do not assert a total order of internal mutations. Original
component indices and withdrawal action sequences remain in the archive.

## Acceptance evidence

Run the existing components once at this installed version pair, then join
actual JUnit outcomes to every S01–S08, P01–P10, G01–G08 and C01–C08 obligation:

```sh
.venv-offline/bin/python -m pytest offline/tests --basetemp=test-evidence --junitxml=test-results.xml
.venv-offline/bin/python -m pytest shared-core/tests/shared_control --junitxml=server-test-results.xml
.venv-offline/bin/python offline/tools/acceptance_matrix.py \
  --server server-test-results.xml --runtime test-results.xml \
  --output acceptance-matrix.json
```

The matrix contains each selected test/parameterization, duration, report
reference and evidence level. Missing/skipped/failed required selectors and
any failing suite case fail aggregate acceptance. Component fixtures that stub
numerical validation remain component evidence; `test_integration.py` uses
real receipts and solver/evaluator processes throughout. Linux CI uploads the
matrix, both JUnit reports, all loop/replay evidence, detailed component traces,
wheels/checksums, dependencies and exact revisions.

| Evidence | Coverage |
| --- | --- |
| Integration normal/golden/reversal tests | Alpha zero, positive residual move, within-tolerance zero penalty, immediate current-direction support, accepted smoothing history, independent plant lag |
| Integration fault tests | Release, re-engagement, expired/duplicate/old/forged results, stale sources, host rejection, runtime division error, real SIGSTOP worker stall, context reinitialization and explicit fault rearm |
| Existing runtime tests | Foreign source clocks, old sessions, exact stale-age boundaries, atomic bundle replacement, host constraints, missing feedback stop, controller submission failure |
| Existing producer tests | Structure/numerics, graph identity, forbidden source, units, receipts, interpreter faults, isolation, infeasible solver holds and immutable archive corruption |
| CLI archive and fresh-process replay | Installed pair, full immutable graph/resources, accepted commands versus proposals, raw residual/penalty reproduction at atol=rtol=1e-9 |

P05 capture-time TF adapter behavior is explicitly deferred; no schema test
substitutes for it. P08 establishes only synthetic joint-feedback provenance;
real sensor/adapter provenance and hardware FK/TF comparison remain required.
Actual tracking, physical stopping, transport behavior and comparative research
benefits remain outside this phase.

Reversal has its own analytical integration oracle. A reversal proposal can
correctly fail the live gate if the previous target moves the plant far enough
during the solve; the gate never relaxes its fresh-state step bound to match a
pinned-state solver result. The normal CLI schedule uses constant right input.

The default loop uses two programs (single-relation golden variants use one).
It does not raise the profile program cap or budgets. The accepted scaling
limitation remains [semantic-shared-control #30](https://github.com/ZenWang00/semantic-shared-control/issues/30).
Loaded hosts may time out even a small solve; timeout remains a targetless hold.
CI evidence identifies its actual Linux platform, not the local reference VM.
