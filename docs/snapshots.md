# Immutable snapshots and local dispatch

`ssc-runtime==0.2.0` implements P1-09 / [issue #2](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/2).
It consumes `ssc-core==0.6.0` at
`95d618f567f5f057a1b3f1737cc394a105fa9a4e` and the ActiveBundle API delivered at
runtime revision `ef039bd139f5caff03b38cd39f843ea8d326bc30`. Shared schema 1.0,
geometry-residual-v1 and the
[offline-reference-v1 profile](https://github.com/ZenWang00/semantic-shared-control/blob/95d618f567f5f057a1b3f1737cc394a105fa9a4e/docs/wiki/Offline-Reference-Profile.md)
remain unchanged. These are local Python interfaces, without ROS or devices.

## Host boundary and clocks

Construct one `SnapshotCoordinator` per session with its `ActiveBundleStore`,
an injectable `clock_ns()` returning nondecreasing nonnegative integer
nanoseconds, an explicit clock domain/epoch ID, two trusted adapter source IDs,
and the calibrated joystick mapping ID. Constructor identities come from the
host, never from received messages. The lifecycle owner supplies the current
`control_epoch` and calls `set_control_epoch()` only for increasing cancellation
generations. Input reception and bundle activation do not change it.

`receive(message)` parses and detaches a current-session JoystickSample or
RobotState envelope, registers its immutable identity, and records the original
acquisition time and local receipt time in `TimedInput`. Acquisition stamps must
already be in the declared local domain. Foreign domains, future samples,
replayed/nonincreasing sequences, reversed acquisition times, wrong sources and
wrong joystick mappings are rejected. A clock-domain string is an assertion
from a trusted adapter, not proof of clock synchronization. This component
implements no cross-clock mapping or bounded-delay acquisition protocol.

Production timestamps may use a remote clock; they are retained but never used
to calculate source age. A new source sequence with an old acquisition time is
retained as stale and cannot refresh freshness. Missing input is an error.
Source age equality at the configured timeout is valid. State frames must match
the acquired bundle. The shared parser excludes commanded-pose feedback, but
actual sensor/FK provenance remains the adapter's responsibility. Clock rollback
or a noninteger clock reading disables further timed operations on this
coordinator; a new valid session is required.

`current_inputs()` returns the fresh current `(joystick, robot_state)` pair with
their sampling/receipt records without allocating a snapshot or command. Neutral
and disabled joystick input remains visible to the future gate. This accessor
performs the same freshness/frame checks as assembly and grants no lasting lease.

## Assembly and immutable worker inputs

`assemble()` acquires one bundle, selects the latest valid stream records,
checks acquisition freshness and frames, and forms a schema-valid
`FrozenSnapshot`. Its `snapshot` is the unchanged public ControlSnapshot.
`resolved` is a shared `ResolvedSnapshot` containing the exact bundle, messages,
history and tuple-valued feature arrays. No worker lookup or mutable data source
is needed. The shared registry also retains the snapshot for later resolution.

The wrapper retains the activation identity through `bundle`, session, clock,
original sampling/receipt records, `created_ns`, and `expires_ns`. The expiry is
fixed at creation plus `max_command_age_s`; `dt_s` comes from
`control_period_s`, 0.2 s in the reference profile. Duration settings must be
representable as positive whole nanoseconds. Snapshot and command counters are
independent. Unused snapshots therefore do not consume command sequences.

History contains at most 32 earlier joystick samples, excluding current input,
with acquisition timestamps in the inclusive interval
`[creation_time - history_window_s, current_sample_time]`. Increasing source
sequence breaks equal-timestamp ties. New samples evict older ones by count;
assembly also filters by time. Neutral/disabled samples remain available as
evidence; the estimator decides their contribution. The current sample alone
determines the wrapper's `motion_eligible` flag. This flag records input
eligibility at assembly, not a continuing motion permission. Neutral/disabled
snapshots can be inspected but cannot be dispatched for assistance.

Frozen records and tuple arrays survive subsequent input, geometry, config and
activation updates. `lookup_snapshot(snapshot_id)` returns the original wrapper.
Store currency checks are instantaneous, not leases over later invalidation.

## Dispatch, retry and result receipt

`dispatch(frozen)` accepts only an original snapshot owned by this coordinator.
It checks context identity, epoch, original expiry, pinned and current input
freshness, current state frames, and current nonzero/enabled input. Normal newer
nonzero joystick input does not cancel a bounded calculation by sequence
equality. Release handling still belongs to lifecycle/gate integration.

On first dispatch it allocates a new host `command_seq` and returns an immutable
`DispatchRecord` with the complete inputs, `dispatched_ns`, and:

```text
expires_ns        = created_ns + max_command_age_ns
solve_deadline_ns = min(dispatched_ns + solve_budget_ns,
                        expires_ns - transport_allowance_ns)
```

The optional local transport allowance defaults to zero for offline execution.
Configured solve plus transport allowance must fit the original age budget.
Delayed dispatch truncates remaining solve time. No dispatch is permitted when
the remaining budget is zero. A retry before the original solve deadline returns
the same envelope, sequence and deadlines. At/after either deadline it fails;
retry never restarts computation time. `lookup_dispatch(snapshot_id)` retains
the original record even after expiry or cancellation.

`allocate_local_command_sequence()` reserves an independent sequence for a
downstream local hold; it neither sends nor accepts a command. The worker echoes
the dispatch sequence, epoch and source sequences in its ControlResult.
`observe_result(message)` checks those causal links and move horizon, then stamps
arrival locally. Its `before_expiry` and `before_solve_deadline` booleans report
timing only. Duplicate or late arrivals cannot mutate dispatch or accepted history.
They do not refresh a watchdog. The eventual gate must independently reject old
authority, duplicate/out-of-order results and stale input, and check host
constraints. The solver supervisor must also enforce real monotonic execution
budgets: a fake clock is no evidence of real-time isolation.

## Accepted-command history

Only the trusted local gate/adapter calls `record_acceptance(result_message,
acceptance_record)` after its decision/application. This is an input port for
downstream work, not an implementation of the gate. Never expose it as a worker
message handler. It checks session and result identity, local check time,
accepted/applied-ID coherence, sequence/epoch provenance, unique applied/result
IDs, and original dispatch validity. A local hold must use a sequence reserved
in its current epoch. Rejected decisions return `None` and do not replace the
last accepted command. A result observed without an acceptance never enters
history, even if its solver reports success.

`AcceptedCommand` preserves the result, decision, and exact source snapshot
with measured source state and horizon. `lookup_applied(id)` retrieves only
registered accepted history; missing IDs are errors. The next snapshot's
`previous_applied_command_id` and `previous_applied` come from that history.
A last accepted hold remains the last command; it is never skipped to revive an
older move. A legitimate prior-epoch command remains available for the estimator
to apply the profile's zero-reference rule. These are accepted target reference
velocities, not measured motion or proof of physical tracking.

All coordinator operations serialize through a lock. Concurrent retries allocate
one dispatch sequence. Store artifact access follows its lock contract; callers
must not hold the store artifact lock while waiting for another coordinator
operation. Context validity and motion authority must still be coordinated by
the downstream gate. Session records are retained for evidence/lookup until
session retirement; bounded live history does not imply bounded archive memory.

## Verification and handoff

Run the install and test commands in [offline/README.md](../offline/README.md).
Tests use an integer fake clock beginning at 10 s and the real shared validators,
registry and host receipt trust. Numerical probe success is stubbed for these
snapshot cases; the existing Linux activation integration test still exercises
the real producer workers. CI installs both wheels, checks dependency consistency,
and retains exact source revisions, wheel hashes, JUnit, concurrency evidence and
`snapshot-trace.json` under its `offline-runtime-validation` artifact.

| Acceptance/scenario | Component evidence |
| --- | --- |
| Coherent immutable inputs and P07 prerequisites | exact bundle/program/binding refs; shared re-resolution; detached messages/arrays; context replacement retains old inputs |
| P06 and P09 | remote production-clock offset; foreign acquisition-clock rejection; session/source/sequence/time replay rejection |
| P10 prerequisites | missing/current/stale-at-receipt inputs, equality at timeout, state frames, neutral/disabled input |
| History and dt | empty initial history, equal-timestamp ordering, latest 32, inclusive time boundary, 0.2 s horizon |
| C03 prerequisites | delayed dispatch, reserved transport budget, immutable expiry, retry at solve/expiry boundary, late receipt |
| C04 registry prerequisites | host result echo checks, independent snapshot/command counters, concurrent retry, duplicate/out-of-order acceptance bookkeeping |
| Applied-command provenance | solved/rejected exclusion, accepted lookup, source-state retention, hold preservation, prior epoch and inconsistent acceptance rejection |

The shared estimator (#12) consumes `FrozenSnapshot.resolved` and
`previous_applied`; command acceptance (#3) consumes dispatch, current local
inputs and lifecycle authority, and feeds its trusted acceptance decision;
archival/replay (#17) consumes immutable records and timing evidence. No complete
P06–P10/C03–C04 gate scenario, physical stopping behavior, robot integration or
research benefit is established by these component tests.
