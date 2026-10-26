# Local command acceptance and synthetic constraints

`ssc-runtime==0.3.0` implements [P1-11 / issue #3](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/3).
It consumes `ssc-core==0.6.0` at
`95d618f567f5f057a1b3f1737cc394a105fa9a4e`, retaining schema 1.0,
geometry-residual-v1, and the accepted
[offline-reference-v1 profile](https://github.com/ZenWang00/semantic-shared-control/blob/95d618f567f5f057a1b3f1737cc394a105fa9a4e/docs/wiki/Offline-Reference-Profile.md).
This is synthetic component evidence with a recording adapter. No real controller,
ROS graph, plant, full lifecycle, physical stopping or thesis benefit is implied.

## Pure checker for optimizer injection

`SyntheticHostChecker()` implements the server's `HostConstraintChecker` protocol.
Pass it to the future shared optimizer; the server never imports this repository.
It reads only supplied immutable pose/state/configuration values. Its finite,
dimensionless named margins cover workspace faces, measured/candidate inverse
joint bounds and clearance over the entire straight segment from measured TCP
to candidate TCP. A zero-length segment checks its point. It also rejects wrong
model/frame/policy identities, nonnormalized/nonfinite poses, incorrect joint
names/order, inverse-domain failures and inconsistency between measured q and TCP.
The synthetic adapter uses joint order q1 through q6, all in radians.

`proxy_forward()` and `proxy_inverse()` provide the profile's rotary-coordinate
proxy and principal inverse. Inverse coordinates are never clipped. Inverse
joint bounds are reported by the checker; obtaining an inverse is not feasibility.
`basic_step_constraints()` separately returns the input-scaled translation and
rotation norm margins, current-input half-space, and all six component bounds.
It uses control-frame SO(3) increments and the unchanged candidate horizon.
Both reports use numeric-offline-v1's `-1e-8` hard margin tolerance. The model
agreement tolerances are `1e-9 m` and `1e-8 rad`. Callback errors reject the target.

## Host authority and result submission

Construct one `CommandGate(coordinator, adapter, estimator_config_id=...)` per
`SnapshotCoordinator`. The default installed estimator identity is
`intent-linear-v1`; a different approved estimator must be selected by the host.
The gate starts with motion and controller readiness disabled. Only the trusted
lifecycle owner calls `set_authority(control_epoch=..., motion_enabled=...,
controller_ready=...)`. Its epoch must equal the coordinator's current epoch.
Neither an input sample, context activation nor worker result grants permission.

`submit(result_message, weights=weights_message)` is the worker entry point.
Weights may instead have been registered previously. The gate parses untrusted
records and resolves the exact weight/snapshot/estimator/candidate identities,
program coverage and binding normalization, including finite penalty consistency.
For moves, evaluation must be valid, solver status solved, and horizon equal to
the dispatched snapshot's dt. A positive generated residual remains eligible;
it does not relax independently recomputed host constraints.

Before any adapter action the gate checks the current session, epoch, exact
activation identity, dispatch-issued command sequence, last accepted sequence,
last accepted move snapshot sequence, original expiry, fresh current joystick
and measured feedback, current permission and controller readiness. Worker input
sequence echoes must match pinned inputs. They need not equal newer live sample
sequences: ordinary nonzero input updates preserve eligibility, but their current
magnitude/direction and current measured state determine the final hard margins.

`arrivals` retains locally stamped `ResultArrival` values. For this local-only
boundary, a move must arrive before its original solve deadline; a remote
`produced_at` or declared duration is not completion proof. Original expiry is
checked again at final submission. Registry/preflight work may consume the
remaining age budget but never restarts either deadline. A timely arrival can
still be accepted after the solve deadline, provided final acceptance is before
snapshot expiry. Separate transport completion proof is future integration work.
An error/timeout hold may arrive at or after the solve deadline, when the worker
supervisor reports its failure, provided the original snapshot has not expired.
It cannot carry an over-budget successful solve or extend the snapshot deadline.

Unknown/old sessions, epochs, contexts, forged sequences, duplicate/older command
or snapshot order, and expired results are discarded. The same rules apply to
worker holds. Discards do not cancel the current target, change accepted history,
or refresh `last_eligible_result_ns`. Malformed records with a demonstrably
current host allocation trigger local failure handling; malformed records whose
authority cannot be established are only discarded. No malformed field is repaired
into an executable result.

## Adapter action, failures and evidence

The trusted `CommandAdapter` implements synchronous, bounded local `apply`,
`cancel`, `hold` and `stop` submissions. No worker waits, network I/O or runtime
reentry is permitted inside these methods. Device integrations must implement
that boundary separately. A successful return means a request was submitted;
`GateDecision.observed_execution` remains null. Actual feedback/acknowledgments
must be archived separately by the controller/plant integration.

`apply(AcceptedTarget)` receives the proposed result, current input and measured
state, acceptance time and fixed target expiry `acceptance + horizon`. Only
successful submission enters `AcceptedCommand` history. An eligible worker hold
calls cancel then hold at fresh compatible measured state, or stop if feedback
cannot support a hold. It preserves its causal snapshot and dispatch sequence.

A current invalid move receives a rejected `CommandAcceptanceRecord` with no
applied ID. The gate also records a distinct trusted local hold with a newly
allocated host sequence, null snapshot/weights and its own acceptance record.
Thus a rejected proposal never becomes the smoothing reference. `local_hold(reason)`
provides the same immediate trusted path to the lifecycle owner, independently
of worker completion or joystick freshness. Local holds do not renew worker
liveness. Eligible accepted worker moves/holds update `last_eligible_result_ns`;
the engagement/watchdog timer itself belongs to issue #4.

Current failures and holds interlock further motion and request `PAUSED` recovery.
Controller readiness failure or any failed adapter action requests `FAULT`.
Failed cancel/hold attempts fall back to stop. An adapter failure never yields
acceptance of the failed proposal; only a separately successful fallback may
enter history. The lifecycle owner must advance the epoch, cancel pending work,
process the requested transition and enforce neutral/re-engagement or explicit
fault rearm before calling `set_authority` again. Calling `set_authority` alone
does not implement these recovery rules.

## Serialization and downstream handoff

Registry preflight happens before the final action transaction and holds no
coordinator/context lock. Withdrawal, local hold and invalidation can proceed
while it waits. The gate then rechecks live conditions inside
`SnapshotCoordinator.command_transaction()`, locking coordinator then context
through final checks, adapter submission and accepted-history commit. Direct
`receive`, epoch updates, bundle begin/publish/invalidate and concurrent results
cannot interleave between those final checks and submission. Artifact locks are
never acquired within this transaction. Numerical preparation is still a
motion-disabled operation, as required by ActiveBundleStore.

Worker results are preflighted in the immutable registry. Gate-local holds and
their source-free provenance are retained in accepted history and `decisions`;
they need no registry lookup. Archive these records as well as worker arrivals,
proposals and weights. `previous_applied_command_id` continues to reference only
accepted history; a hold is never skipped to revive an older movement.

The caller's event loop must process release/disable/invalidation before a
same-tick result, then enforce target expiry/watchdogs per the reference profile.
Locks establish serialization, not operating-system timing guarantees. Lifecycle
issue #4 owns immediate event withdrawal and recovery; plant issue #5 owns target
expiry, independent simulated measurements and actual adapter implementation.

## Verification

Use the clean-install commands in [offline/README.md](../offline/README.md).
CI builds both pinned wheels on Ubuntu/Python 3.12, runs all runtime tests plus
the existing real producer/consumer receipt test, and retains JUnit, dependency
versions, wheel hashes, both source revisions and `gate-trace.json` under
`offline-runtime-validation`. Numerical receipt generation is stubbed in gate
unit cases; the separate Linux integration test retains real-worker coverage.

| Scope/scenario | Component evidence |
| --- | --- |
| C01/C08 and positive soft residual | valid delayed move under newer live input; recorded apply; separate proposal/acceptance/observed execution |
| C03/C04 | fixed expiry and arrival deadline; forged/duplicate/out-of-order commands, older snapshot with larger dispatch sequence; old/duplicate holds |
| P06/P09/P10 gate portion | original sample age versus new receipt/production time; equality boundary; current permission/frames; stale state uses stop |
| C06 adapter boundary | cancel/hold with fresh feedback; stop without usable feedback; adapter failures request FAULT |
| Host constraints | analytical proxy round trip; workspace faces; full sphere crossing with valid endpoints; state consistency; input magnitude/reversal; rotation bounds |
| Concurrency | duplicate completions apply once; registry work cannot block local hold; final context/action serialization |

These tests do not complete downstream lifecycle/plant or aggregate loop
acceptance. Physical limits, collision certification and hardware stopping remain
outside this synthetic profile.
