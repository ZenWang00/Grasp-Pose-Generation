# Immutable snapshots and explicit selection history

`ssc-runtime==0.8.0` uses schema 2.0 `ControlSnapshot` with `candidate_set_id`,
`previous_selection_id` and `previous_applied_command_id`. These are separate:
selection history decides which candidate to assist; accepted-command history
provides translation smoothing. Neither is measured robot motion.

`SnapshotCoordinator` owns a nondecreasing integer-nanosecond local clock, its
clock domain, trusted joystick/state sources and the XY mapping. Input acquisition
must already use this domain. Foreign or future sample times, wrong sources,
replayed/nonincreasing sequences, reversed acquisition and wrong mappings are
rejected. A newer but stale sample stays stale; production or receipt time cannot
refresh it. Missing measurements never become zero. Clock failure requires a new
session. Feedback must be measured or FK from measured joints, in the active frames.

`assemble()` acquires and pins one bundle, checks current acquisition freshness,
and records the exact messages, up to 32 earlier joystick samples, creation time
and fixed expiry. History is bounded by the configured one-second window and
never includes current input. The 0.2-second horizon comes from configuration.
Neutral input remains visible and makes a snapshot ineligible for motion.

After estimating intent, the loop registers a `CandidateSelection` and calls
`record_selection`. The next snapshot references that immutable record and its
source snapshot. Registry and estimator verify earlier sequence, same epoch and
same configuration/task/geometry/semantics/candidate set. Release, disable, epoch
change or context replacement clears selection history. The retained archive
still contains earlier records. Unregistered, stale or released selections cannot
restore live history.

`dispatch(frozen)` allocates a command sequence once and fixes the original solve
deadline. A retry returns the same dispatch; it cannot renew either budget or
expiry. Current bundle, epoch, enabled input and source freshness are rechecked.
The real worker deadline is mapped once to monotonic time at dispatch. Result
arrival records local receipt time and echoed source sequences; it does not grant
acceptance or extend validity.

Only a trusted local adapter decision can populate `previous_applied_command_id`.
Solved or rejected proposals cannot become smoothing history. The accepted result,
its acceptance record and exact source remain available for replay. Local holds
use host-reserved sequences without inventing a worker snapshot. Counter ordering,
original expiry and unique acceptance identities are enforced.

The command transaction serializes final gate actions with input, epoch and bundle
changes (coordinator lock before context lock). Registry work and worker waits must
complete outside that short action guard. Snapshot data itself never authorizes
motion; lifecycle owns engagement and the gate makes the final live decision.
