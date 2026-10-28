# Local command acceptance and synthetic constraints

`ssc-runtime==0.8.0` accepts schema 2.0 candidate-position proposals. The host
remains final motion authority; a solved target and a selected grasp point do not
bypass it. No real controller or hardware validation is claimed here.

`SyntheticHostChecker` receives explicit immutable state/configuration and checks
workspace, measured and candidate inverse joint bounds, and clearance over the
whole straight segment from current measured TCP to the proposed TCP. It rejects
invalid model/frame/policy identities, joint names, inverse domains and disagreement
between joints and TCP. The model is the declared six-coordinate rotary proxy.
Agreement tolerances are 1e-9 m and 1e-8 rad; hard margins allow -1e-8.
`basic_step_constraints` independently checks input-scaled norm, component bounds
and the live XY input half-space. Callback errors reject the target.

The trusted lifecycle calls `set_authority`, `local_hold` and `local_stop`.
Worker transport calls `submit(result_message, selection=selection_message)`.
The selection can alternatively have been registered earlier. Gate preflight:

- Resolves the exact snapshot, candidate set and host-approved intent configuration.
- Recomputes candidate scores and selection from the pinned inputs and predecessor.
- Checks that the one diagnostic candidate is the selected real point.
- Recomputes distance, fixed scale/tolerance, penalty and satisfaction at the proposed position.
- Requires solved/evaluation-valid moves with the exact dispatched horizon.

A positive distance residual is valid soft assistance, not a hard feasibility
failure. No weighted average point is generated. Before apply, orientation must
be equivalent to the **pinned source measurement** within 1e-8 rad; quaternion
signs are equivalent. Separately, host feasibility and step limits use **fresh
current measured feedback**. Thus neither source substitution nor fresh-feedback
checks erase the orientation-hold requirement.

Before any action the gate rechecks session, epoch, current activation, issued
command sequence, previous accepted sequence/snapshot, original expiry, original
solve deadline, live input and state freshness, permission and controller status.
New nonzero input does not automatically cancel work; its current magnitude and
direction constrain the final motion. Remote timestamps never prove freshness.
Old, duplicate, forged-sequence or expired results are discarded without canceling
a newer target. A current attributable failure takes the separate local hold path.

The adapter must implement nonblocking `apply`, `cancel`, `hold` and `stop` without
reentering runtime. Hold uses fresh valid measured state; stale or invalid feedback
selects stop. Failed cancel/hold falls back to stop and requests FAULT. Successful
submission is not observed physical execution. Only accepted commands enter
smoothing history. Decision and arrival records retain original timing and causes.

Tests exercise real candidate metrics, stale/source mismatch, hostile result IDs,
constraint failures, orientation hold, equivalent quaternion signs, expired work,
lock ordering, adapter exceptions and unchanged independent plant feedback.
