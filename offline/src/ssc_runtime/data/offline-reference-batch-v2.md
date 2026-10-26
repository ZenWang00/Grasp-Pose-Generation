# Offline Batch Profile

**offline-reference-batch-v2 · 26 October 2026 · P1-17**

This profile is the explicitly selected candidate-batch extension of
[offline-reference-v1](Offline-Reference-Profile.md), whose content is retained
unchanged. The original profile at source
`9ce2d2467f10ca6d82e7672a3158aa6f584ebfde` remains the complete definition of
the scene, estimator, SLSQP mathematics, host constraints, plant and comparison
tolerances. This extension changes only the solver's evaluation topology and
the corresponding execution/configuration identities. The owner requested
implementation of this concrete proposal in conversation
`01a179d2-c7ae-7513-93b4-330e50314727`; the durable decision and acceptance
record is [Issue #30](https://github.com/ZenWang00/semantic-shared-control/issues/30).

## Identities and compatibility

| Identity | Content |
| --- | --- |
| `offline-reference-batch-v2` | Original v1 profile plus the explicit changes on this page |
| `solver-slsqp-batch-v2` | Unchanged SLSQP numerics/limits; one isolated evaluator process for a complete candidate batch |
| `config-offline-batch-v2` | Original session values, paired with this profile and batch solver |
| `config-offline-batch-alpha0-v2` | The same session values with zero assistance |
| `intent-linear-batch-v2` | Original `intent-linear-v1` values, bound to `config-offline-batch-v2` |
| `intent-linear-batch-alpha0-v2` | Original `intent-linear-alpha0-v1` values, bound to `config-offline-batch-alpha0-v2` |

The session record has its new configuration ID; no wire field is added.
The host installation and run manifest explicitly select the solver ID.
`numeric-offline-v1`, `geometry-residual-v1`, schema 1.0,
`synthetic-host-v1` and `synthetic-rotary-v1` remain unchanged. A changed binding
uses the new configuration ID and its own immutable identity. Existing
`solver-slsqp-v1` content, default lookup and old archives remain supported.
The new topology is opt-in through
`get_solver_config("solver-slsqp-batch-v2")`; upgrading the library alone does
not silently migrate an installation.

## Execution boundary and budgets

The host still supervises one separately spawned solver process with a 1 GiB
address-space limit. The solver prepares one separately spawned evaluator
process, with a 256 MiB address-space limit, containing 1–16 individually
validated/compiled restricted programs. Generated source never executes in
the host or SciPy solver process. Both processes disable core dumps and retain
the existing BLAS thread limits and 1 MiB bounded JSON IPC ceiling.

For the first candidate of each solve, the solver submits the complete ordered
program set, resolved keypoints/axes/parameters and that pose in one request.
Subsequent candidates send the pose and the same unique solve-request identity,
referencing only that solve's validated immutable inputs. The evaluator
checks exact program identities/order and each input shape/domain, then
evaluates every program. A successful reply contains exactly one finite scalar
per program, in the same order. A malformed/missing/reordered reply or any
program failure invalidates the complete candidate; partial results never
enter the objective or a move proposal. Programs are evaluated even at zero
assistance. Final residuals are recomputed from the emitted target.

The common pose is checked once per batch using the scalar interpreter's exact
conversion and rotation checks. The evaluator retains only the latest solve's
validated immutable binding inputs. Missing or mismatched solve references fail;
each new solve supplies and validates its full bindings again. Boolean values
cannot alias numeric inputs. Every program expression still runs
for every candidate batch, including the final emitted-target recomputation.
Inside the already supervised solver process, serial batch exchanges run
inline with the same deadline/cancellation/retirement machinery. They do not
create an extra supervisor thread per candidate. Host-facing operations remain
asynchronous, and generated programs still execute only in the child evaluator.

**The complete batch has a 10 ms real-monotonic deadline**, including host
serialization, all program evaluations and reply publication. It is further
shortened by the remaining solve deadline. The budget is not multiplied by
program count. This deliberately replaces the v1 per-program IPC deadline with
a stricter whole-candidate deadline under a new solver identity; the scalar
evaluator used for receipts and replay keeps its original 10 ms behavior.

The maximum remains 16 programs and the total solve budget remains 150 ms,
including input validation, IPC, optimization and host recheck, additionally
bounded by the original snapshot expiry. SLSQP remains serial, with zero
initialization, `jac=None`, `eps=1e-6`, `ftol=1e-9`, `maxiter=100` and the
unchanged objective, finite differences, hard margins and tolerances.

Timeout/memory-limit/stall means a targetless `timeout` hold and worker
retirement. Domain/protocol failures mean a targetless error hold and retirement.
The whole solver process group remains subject to the host's bounded teardown.
Preparation/re-preparation occurs only while motion is disabled. Gate authority,
release, watchdogs, failure recovery and command age do not change.

## Reproducible program-count acceptance

`docs/reference/verify_program_budget.py` runs the installed wheel on Linux.
For **every integer count 1–16**, it retains all 20 reversal solves and all 20
infeasible-start solves (640 total). Replicated relations have distinct program
identities and per-object binding weights summing to one, so scaling does not
weaken the original fixture by changing tolerance or total assistance weight.
The one-program reversal uses the bottle candidate; counts 2–16 preserve both
objects. It records status, elapsed time, deadline margin, total submitted
program evaluations, batch IPC calls and per-batch elapsed statistics.

Every sample must complete with the expected `solved` or `infeasible` result
before 150 ms; timeout is a failed performance criterion, even though it remains
a correct safety outcome. Failures are retained, never retried as replacement
samples. A retired worker may be re-prepared for the next numbered sample.
The report includes medians, p95, maximum, timeout counts, source/wheel identity
and actual machine/software environment. CI also runs numerical, isolation,
fault, archive and legacy-replay checks.

These are measurements on the stated host, not guarantees for arbitrary
generated programs, machine load or hardware. Stronger statements would require
a different experiment. P05 capture adapters, P08 real measured-state provenance,
physical stopping and research benefits remain outside this engineering task.

## Consumer and evidence handoff

The robot-owned runner selects the new solver/configuration IDs and pins the
accepted producer commit. The full producer/consumer pair is checked with the
existing 34-scenario matrix, real receipts, independent synthetic feedback,
archive integrity and fresh-process replay. The archive retains both the frozen
base profile and this extension; old model/host resources keep their original
bytes. The Issue/PR delivery records contain the measured result, exact tested
revisions, independent review and CI artifacts; this specification alone is
not proof that the acceptance run passed.
