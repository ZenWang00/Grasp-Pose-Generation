# Lifecycle, withdrawal and watchdogs

`ssc-runtime==0.4.0` implements [P1-13 / issue #4](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/4).
It consumes `ssc-core==0.6.0` at `95d618f567f5f057a1b3f1737cc394a105fa9a4e` and the
accepted [offline-reference-v1](https://github.com/ZenWang00/semantic-shared-control/blob/95d618f567f5f057a1b3f1737cc394a105fa9a4e/docs/wiki/Offline-Reference-Profile.md)
settings (sections 2 and 5). It implements
[Runtime Lifecycle §§1,5–7](https://github.com/ZenWang00/semantic-shared-control/blob/43cf81a942dafa654f7c7e65acf2f7a295dea847/docs/wiki/Runtime-Lifecycle-and-Timing.md)
for the offline core. Evidence is software simulation with a controllable clock
and a recording adapter. No controller stopping time, physical safety, plant,
transport or hardware behavior is established here.

## Ownership

`LifecycleCoordinator` is the only caller of `SnapshotCoordinator.set_control_epoch()`
and `CommandGate.set_authority()/local_hold()/local_stop()`. It owns the session,
`control_epoch`, lifecycle state, engagement, watchdogs and recovery. Workers,
activation callbacks and supervisor signals never grant motion. A trusted host
session factory creates each session's `ActiveBundleStore`, `SnapshotCoordinator`
and `CommandGate`; the lifecycle rejects a runtime that is mis-wired, already
active, reuses a session ID or does not start with fresh counters.

| Entry point | Caller | Effect |
| --- | --- | --- |
| `on_input(message)` | joystick / measured-state adapters | Receives the sample; releases, engages or recovers on the local path |
| `tick()` | runner, every plant tick | Source freshness, missed result deadline and liveness watchdogs; PAUSED recovery |
| `dispatch_cycle()` | runner | Freezes and dispatches one snapshot, only while ACTIVE |
| `submit_result(result, weights=...)` | worker transport | Gate decision, then the lifecycle request it carries |
| `begin_/prepare_/complete_initialization`, `initialize` | host | Motion-disabled validation and atomic activation |
| `invalidate_context(reason)` | host | Task, scene, semantic, feature or configuration invalidation |
| `controller_status(ready=, fault=)` | controller adapter | Readiness; a fault latches FAULT; a later healthy report clears its cause |
| `computation_status(ready=, reason=)` | solver supervisor | Readiness after motion-disabled preparation |
| `rearm(reason)` | operator | FAULT → UNINITIALIZED, only after clearance |
| `restart_session(reason)`, `clock_domain_reset(reason)` | host | Withdraw, retire records, start a new session |

`WorkerCanceller.cancel(control_epoch, reason)` is a best-effort, nonblocking
request to drop work issued at or before an epoch. Its failure is recorded and
never awaited; gate epoch checks still reject late results.

## Withdrawal

Every exit from READY or ACTIVE (except READY → ACTIVE), every FAULT latch and
every session restart performs one withdrawal, in this order: disable gate
authority and increment `control_epoch`; record the latest input sequence; issue
a trusted local hold with a newly allocated host `command_seq` (cancel, then
hold at fresh feasible measured state, otherwise stop); request worker
cancellation for the previous epoch. FAULT uses cancel then stop. When the gate
has already held on its current-failure path, the lifecycle completes the
withdrawal around that decision without a second hold. A failed cancel/hold/stop
request latches FAULT, even if a fallback stop succeeded. Requests are not
observed stopping: `LifecycleRecord.observed_execution` is always null.

Release or disable is processed inside `on_input` and never waits for a cycle,
worker, registry preflight or numerical preparation. Discarded, replayed, stale
or old-session messages change no state and refresh no watchdog.

## Transitions and recovery

| Event | Immediate behavior | Recovery |
| --- | --- | --- |
| Ordinary release or disable | Withdraw, PAUSED (`release`) | Same bundle, fresh inputs, a neutral sample newer than the withdrawal, eligible feedback → READY; then a fresh engagement |
| Stale joystick/state, host rejection, controller not ready | Withdraw, PAUSED (`input`, `host_rejection`, `controller`) | As for release |
| Worker hold, invalid result, missed result deadline, liveness expiry, supervisor unavailable | Withdraw, PAUSED (`computation`) | As for release, plus a `computation_status(ready=True)` received after the failure |
| Context invalidation, validation rejected or cancelled | Withdraw if READY/ACTIVE, bundle ineligible, PAUSED (`context`) | Reinitialize and activate, then a fresh engagement |
| Initialization requested from READY/ACTIVE | Withdraw, then VALIDATING (via PAUSED from ACTIVE) | Activation, then a fresh engagement |
| Controller fault, failed hold/stop, apply failure, unrecoverable host condition | Withdraw with stop, FAULT latched | Healthy controller report after the latch, explicit `rearm`, UNINITIALIZED, initialization, engagement |
| Session restart | Withdraw, then retire store/coordinator/gate and allocate a new session | UNINITIALIZED; initialization and engagement again |
| Clock failure or clock-domain reset | Epoch/gate withdrawal, direct cancel + stop (freshness is untrusted), new session | UNINITIALIZED; FAULT if the stop request failed |

**Fresh engagement** is a neutral/disabled sample observed while READY (or the
latest fresh neutral sample at READY entry), followed by a newer fresh enabled
nonzero sample, with fresh feasible measured feedback, current context,
controller readiness and computation readiness. A refused attempt disarms and
needs another neutral sample. Activation completing, supervisor/controller
signals and restored feedback never engage a nonzero input that is still held.
Ordinary nonzero updates never change the epoch or require result/live sequence
equality.

**FAULT** is latched. A session restart or clock reset while FAULT keeps the latch
in the new session. Rearm is refused until a controller status report with no
fault and `ready=True` arrives after the latch; a later fault revokes clearance.
Two choices are deliberately stricter than the state diagram: a controller fault
latches FAULT from every state, including UNINITIALIZED, and READY exits also
perform the withdrawal although READY holds no motion authority.

## Watchdogs and event order

`tick()` checks, in order, while ACTIVE: engaged bundle still current; joystick
age; measured-state age (age equal to the timeout is fresh); any outstanding
current-epoch dispatch at or after its original expiry (`missed_result_deadline`);
and liveness, which expires at or after 0.2 s since engagement or the gate's last
accepted current-authority result. While READY it pauses on stale required input.
While PAUSED it evaluates recovery. Arriving samples are checked immediately on
the input path as well.

For equal local timestamps the runner processes withdrawal events (release
arrives through `on_input`; invalidation, faults, restarts), then fresh
input/state, then `submit_result`, then `tick`, then `dispatch_cycle`. An
acceptance may therefore refresh liveness before the watchdog at the same tick,
but cannot rescue a snapshot at its own expiry. These are deterministic test
settings, not operating-system scheduling guarantees.

## Owner decisions

Recorded in the [claim comment](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/4#issuecomment-6418789288):
explicit computation readiness after a computation failure; FAULT latched
across session restarts and clock resets; independent review by a separate agent
before merge. No wire contract, frame, unit or profile identity changed.

## Evidence

`offline/tests/test_lifecycle.py` uses a fake integer-nanosecond clock, a
recording adapter with injectable apply/cancel/hold/stop failures and a recording
worker canceller. `test_retained_lifecycle_trace` writes `lifecycle-trace.json`
(records, adapter calls, worker cancellations, scenario markers, configuration).

| Criterion or scenario | Tests |
| --- | --- |
| Local release/disable; C02 | `test_release_during_stalled_solve_is_local_and_rejects_late_result`, `test_release_fixture_epoch_increments_once_and_resume_needs_fresh_neutral_engagement` |
| Hold at measured state vs stop; C06 | `test_fresh_feedback_holds_at_current_measured_state`, `test_stale_or_invalid_feedback_selects_stop` |
| Source expiry, deadlines, lost progress; P10, C05 | `test_source_age_equality_is_fresh_and_stale_state_withdraws_on_input_path`, `test_stale_joystick_withdraws_and_holds_at_fresh_feedback`, `test_lost_upstream_progress_liveness_watchdog_and_discards_do_not_refresh`, `test_same_tick_acceptance_refreshes_liveness_before_watchdog`, `test_missed_result_deadline_withdraws_before_liveness` |
| Every withdrawal follows §6 | `test_every_authority_withdrawal_follows_its_transition` (14 events) |
| Recoverable input/computation failures | `test_recoverable_input_failure_needs_restored_inputs_neutral_and_engagement`, `test_computation_failure_needs_supervisor_readiness_neutral_and_engagement`, `test_engagement_requires_computation_controller_and_eligible_feedback` |
| Ordinary updates; repeated neutral | `test_ordinary_nonzero_updates_keep_epoch_and_neutral_repeats_add_no_authority` |
| Release vs invalidation; C07 | `test_ordinary_release_reuses_the_valid_bundle`, `test_ready_engagement_with_stale_feedback_is_refused`, `test_context_invalidation_requires_reinitialization_and_new_engagement`, `test_rejected_validation_pauses_until_a_valid_context_is_activated` |
| No asynchronous resumption; late prior authority | `test_activation_and_async_signals_never_resume_held_input`, `test_superseded_or_late_initialization_never_activates`, `test_late_prior_authority_moves_and_holds_do_not_disturb_newer_decisions` |
| FAULT latch and rearm | `test_fault_latches_until_clearance_and_explicit_rearm` (4 causes), `test_fault_reported_again_revokes_clearance`, `test_unrecoverable_host_condition_during_validation_latches_fault` |
| Session restart, clock reset; P09 | `test_session_restart_withdraws_before_clearing_and_old_session_cannot_revive`, `test_replayed_older_source_sequence_changes_nothing`, `test_fault_latch_survives_session_restart`, `test_clock_domain_failure_stops_directly_and_starts_new_session`, `test_explicit_clock_domain_reset_starts_new_session` |

## Handoff to #5 and #6

- The [plant (#5)](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/5) implements `CommandAdapter` as bounded, nonblocking submissions. `cancel` removes the target; `hold` uses the supplied measured state and stops advancement; `stop` clears the target without fresh feedback; targets expire at `accepted_ns + horizon` without renewal. It reports `controller_status`, including faults and post-fault health, and raises on a failed submission.
- The [runner (#6)](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/6) routes all inputs through `on_input`, all worker results through `submit_result`, dispatches only through `dispatch_cycle`, calls `tick` every plant tick in the event order above, wires the solver supervisor to `computation_status` and `WorkerCanceller`, and archives `LifecycleCoordinator.records` with gate decisions.
- Unresolved hardware verification: the real controller stop mechanism and timing, real clock mapping and drift handling, controller fault reporting, and supervisor cancellation latency.
