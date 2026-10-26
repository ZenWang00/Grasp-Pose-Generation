# Synthetic controller and feedback plant

`ssc-runtime==0.5.0` implements [P1-14 / issue #5](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/5),
using `ssc-core==0.6.0` at `95d618f567f5f057a1b3f1737cc394a105fa9a4e`.
The model is `synthetic-rotary-v1`, with `synthetic-host-v1`, `numeric-offline-v1`
and `config-offline-v1` under
[offline-reference-v1 §§2, 5–7](https://github.com/ZenWang00/semantic-shared-control/blob/95d618f567f5f057a1b3f1737cc394a105fa9a4e/docs/wiki/Offline-Reference-Profile.md).
Schema 1.0 and geometry-residual-v1 are unchanged. No ROS, hardware model,
physical dynamics, calibrated measurements or stopping guarantee is provided.

## Interface

`SyntheticController` implements the existing `CommandAdapter`. Construct one
per session/clock domain using the same callable integer-nanosecond clock as
`SnapshotCoordinator`. The default initial joints are six zeros; optional
initial joints must satisfy the synthetic bounds and workspace. The controller
requires the profile's 0.2 s horizon and 0.05 m/s, 0.5 rad/s motion settings.
The caller serializes all operations; this component creates no threads and
makes no callbacks into the lifecycle.

```python
from dataclasses import asdict
from ssc_runtime import SyntheticController, PLANT_TICK_NS

# config is the parsed SessionConfig; clock.now initially equals 10_000_000_000.
plant = SyntheticController(config, session_id="session-offline-v1",
                            clock_ns=clock, clock_id="synthetic-monotonic-epoch-1")
# Wire plant into CommandGate, and use plant.source_id as robot_state_source_id.
initial_feedback = plant.sample()

# On each explicit runner tick, after same-time withdrawal events:
clock.now += PLANT_TICK_NS
plant.advance()
feedback = plant.sample()
if feedback is not None:
    lifecycle.on_input(feedback)
lifecycle.controller_status(**asdict(plant.status))
```

The example assumes the gate/lifecycle have already been constructed. The full
runner and real optimizer integration remain [issue #6](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/6).
The runner must catch `ControllerError` from `advance()` and report
`plant.status` to the lifecycle before any further dispatch. After a controller
fault, report status even if feedback is missing. `CommandGate` already catches
submission failures and requests lifecycle FAULT.

| Operation | Observable behavior |
| --- | --- |
| `apply(AcceptedTarget)` | Install or replace a bounded target; retain the accepted input and fixed expiry. Validate session, increasing command sequence, epoch, timing, input, feedback and constraints. No joint update or new feedback. This trusted API is called only by the gate; it cannot prove gate acceptance from a forged Python object. |
| `advance()` | Exactly one 20 ms step after the previous plant timestamp. Reject repeated/skipped ticks; no catch-up. At expiry equality clear the target before stepping. |
| `sample()` | Emit immutable joint-derived `RobotState` from the last committed tick. Re-reading the same tick returns the same message and source sequence. |
| `cancel()` | Remove the target, with a distinct action record. The next tick has zero velocity. |
| `hold(state)` | Validate supplied fresh compatible feedback, remove the target and zero synthetic velocity. Retain the supplied sample in the trace; never copy its pose into plant state. |
| `stop()` | Clear the target and zero synthetic velocity without requiring feedback. |
| `status` | Return `ControllerStatus(ready, fault)` for the runner to pass to the lifecycle. |

On each live step, compute Cartesian translation and principal SO(3) rotation
errors. A shared fraction is the minimum of 0.5, the input-scaled linear speed
ratio, and the input-scaled angular speed ratio; omit zero denominators. Apply
the rotation increment on the left. Validate the actual step's path, inverse
joint bounds, speed and input half-space before committing the inverse-mapped
joints. An internal violation latches a fault and commits no joint change.
The sampled pose is recomputed with `proxy_forward(q)`, independently of the
commanded target; optional velocities are joint differences divided by 0.02 s.

Feedback retains `state_source=fk_from_measured_joints`, the existing public enum,
with `synthetic_base`, `synthetic_tcp`, joints `q1`…`q6` in radians, and metre TCP
coordinates. This enum does **not** establish real measured-state provenance.
`plant.provenance` and every `PlantEvent` explicitly label software simulation.

## Failure injection and recovery

`set_feedback_mode("missing")` returns no sample; `"stale"` returns the last
emitted sample unchanged (or None if none exists). Neither mode changes joint
evolution, stamps an old measurement with a new time, nor increments its source
sequence. `"normal"` resumes observations of the current committed tick.
Sampling between ticks retains the last tick's time. Target receipt and control
actions never manufacture an observation.

`fail_next("apply" | "cancel" | "hold" | "stop" | "advance")` injects a
one-shot failure at the next named operation. Repeated injections queue a count.
The action is recorded as failed, raises `ControllerError` and latches a fault;
a failed submission does not claim success or discard its previous target.
Faulted ticks cannot advance joints. A failed step consumes its tick and records
unchanged joints. `inject_controller_fault(reason)` independently latches a
controller fault. Stop/cancel remain available while faulted.

`clear_fault()` models explicit simulated repair, requires no retained target,
and never grants motion. Report its healthy status, then use the existing
explicit lifecycle rearm, initialization and engagement path. A successful stop
alone does not clear the fault. Invalid target/hold submissions are rejected
without state mutation; the gate turns adapter exceptions into lifecycle FAULT.

The runner processes same-time release/disable/fault/invalidation before plant
advancement, routes fresh state/input next, then eligible results, expiry and
liveness checks, then dispatch, as required by the profile. In particular,
process release through `on_input` before `advance()`. The plant enforces target
expiry independently on its ticks; the lifecycle owns authority revocation,
feedback freshness, engagement and watchdogs. Old results must pass through the
gate, which discards them before they can cancel a newer target. Recreate the
plant on a new session or clock domain; never rewind an existing plant clock.

## Evidence and downstream handoff

`offline/tests/test_plant.py` exercises target receipt, analytical first-step and
lag oracles, combined speed limits, non-identity orientation, replacement,
cancellation, hold, stop, expiry equality, wire round-trip, missing/stale samples,
invalid commands, internal step violations and deterministic failures. Direct
gate/lifecycle tests exercise actual plant application, release and discarded
late results, plus apply/stop/controller failures and explicit fault rearm.
Their numerical receipt preparation is stubbed; they do not run the optimizer.
The existing Linux real-worker activation test still runs in the full suite.

`test_deterministic_retained_plant_trace` runs identical events twice and checks
exact equality, stronger than the profile's 1e-9 m / 1e-8 rad comparison for this
fixed environment. It writes `plant-trace.json` with configuration, provenance,
expected/actual numerical oracles, accepted requests, independently evolved
joints, sampled messages, event ordering and failed actions. CI uploads this
alongside JUnit results, wheels/hashes, exact runtime/core revisions and the
dependency freeze. Cross-platform bitwise identity is not promised.

The runner should archive `plant.events` and `plant.provenance` alongside gate
acceptance/application IDs, lifecycle events, exact version pair and inputs.
Event targets carry result IDs; join them to gate acceptance records rather than
interpreting a proposal or submission as observed movement. `dataclasses.asdict`
converts events to JSON-ready structures. Event history is in-memory and grows
with the finite test run; persistent immutable run archives belong to P1-15.

This provides P08 **synthetic feedback provenance** and C06 **simulated controller
behavior** only. Physical controller/sensor provenance, actual stop performance,
and complete phase-one acceptance remain outside this component's evidence.
