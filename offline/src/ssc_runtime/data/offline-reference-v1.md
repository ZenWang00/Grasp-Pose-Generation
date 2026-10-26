# Offline Reference Profile

**offline-reference-v1 · 25 October 2026 · Implementation specification and numerical design checks; no runtime component or robot validation.**

This profile selects one deterministic CPU baseline for [Phase 1](https://github.com/ZenWang00/semantic-shared-control/issues/2). It is the deliverable of [P1-01](https://github.com/ZenWang00/semantic-shared-control/issues/3), not an implementation of its downstream cards. Read it alongside the [data contracts](https://github.com/ZenWang00/semantic-shared-control/wiki/Data-Model-and-Message-Contracts), [generated-program contract](https://github.com/ZenWang00/semantic-shared-control/wiki/Generated-Constraint-Program-Contract), and [runtime lifecycle](https://github.com/ZenWang00/semantic-shared-control/wiki/Runtime-Lifecycle-and-Timing).

The owner selected a simple explicit estimator, the existing local Linux VM as the validation environment, and nonnegative projection of each next XY step onto current input. These decisions are recorded in the [execution claim](https://github.com/ZenWang00/semantic-shared-control/issues/3#issuecomment-6401653378). Public schema 1.0, geometry-residual-v1, frames, units, release/fault recovery and final robot-side authority remain unchanged. No online perception, real-arm model, ROS graph, transport, hardware tuning or comparison experiment is included.

## 1. Ownership, installation and identities

The server repository owns the transport-independent shared library: contracts, immutable resolution, numerical interpreter/isolation, fusion, optimizer and archive/replay. Grasp-Pose-Generation owns the synthetic host checker, controller/plant, coordinator, gate and complete offline runner. Its future runner installs a pinned server wheel; neither the shared library nor its tests import the robot repository.

The planned distribution is ssc-core, initially 0.1.0, with import namespace ssc_core. It is not published by this document. Its runtime dependencies are NumPy 1.26.4 and SciPy 1.11.4; standard-library-only contract/interpreter code must not import the legacy service at package import time. A future wheel installation in a clean Python 3.12 environment must import without ROS, CUDA, Torch, Open3D, model downloads or service credentials. Do not use the inherited requirements.txt as the shared package's installation requirements. pytest 7.4.4 is the reference development dependency, not a runtime requirement.

The inspected numerical environment is Ubuntu 24.04.4 LTS, aarch64, Linux 6.17.0-35-generic, Python 3.12.3, NumPy 1.26.4 and SciPy 1.11.4. It is a local VMware VM. These existing dependencies passed the calculations in section 7 and were not upgraded. Other platforms may perform development checks; they are not the resource-isolation acceptance environment for this profile.

| Immutable identity | Defined content |
| --- | --- |
| offline-reference-v1 | This complete profile, including fixture and comparison rules |
| config-offline-v1 | Session values in section 2 and the default component identities below |
| intent-linear-v1 | Section 3, alpha_active = 0.5, and the fixture object-to-approach-point mapping |
| solver-slsqp-v1 | Section 4, including smoothing and termination |
| numeric-offline-v1 | Section 6, including all probes, tolerances and limits |
| synthetic-rotary-v1 | The model, controller and plant in section 5 |
| synthetic-host-v1 | Workspace, excluded sphere, path and step checks in section 5 |
| bottle-cup-offline-v1 | Default and explicitly named fixture variants in section 7 |

Changes to any content require a new applicable identity; never update content behind an old ID. For the alpha-zero oracle use intent-linear-alpha0-v1 and config-offline-alpha0-v1, with all other values unchanged. Each named fixture variant is a separate immutable record. Records remain scoped to their session according to schema 1.0; profile/model definitions can be referenced as immutable configuration resources. No new public wire field is introduced.

A future integration manifest must record the exact server and ROS2 Git SHAs, built wheel version and SHA-256, dependency lock, Python/OS/architecture, schema/ABI, every profile/configuration/model/policy/fixture ID, and result/archive references. The pinned baseline ROS tag is not an accepted integration pair. At this stage shared_package_version and tested_integration_pair are unavailable, not invented values. The isolated calculations identify their script and result artifacts separately.

## 2. Synthetic session values and clocks

| SessionConfig field or associated host setting | Reference value |
| --- | --- |
| control_frame / tcp_frame | synthetic_base / synthetic_tcp |
| robot_model_id / validation_policy_id / motion_constraint_set_id | synthetic-rotary-v1 / numeric-offline-v1 / synthetic-host-v1 |
| control_period_s; snapshot dt_s | 0.2 s |
| max_xy_speed_mps; max_linear_speed_mps | 0.05 m/s; 0.05 m/s |
| max_angular_speed_radps | 0.5 rad/s |
| joystick_timeout_s; robot_state_timeout_s | 0.5 s; 0.5 s |
| solve_budget_s; max_command_age_s | 0.15 s; 0.2 s |
| history_window_s; additional count bound | 1 s; 32 earlier samples |
| max_capture_skew_s | 0.01 s, metadata checks only |
| synthetic plant/input/feedback tick | 0.02 s |
| eligible-result liveness watchdog | 0.2 s since engagement or last accepted current-authority result |
| controller target expiry | acceptance time + horizon_s, never renewed by re-reading a target |
| worker preparation timeout | 5 s while motion is disabled |

All values are synthetic settings, not measured control rates, robot limits or deadline guarantees. Use integer nanoseconds for the controllable local test clock. A fixture begins at 10 s in one declared synthetic clock domain; plant feedback and joystick sampling advance on its 0.02 s ticks. Source age equality is allowed at its threshold; an acceptance at or after the fixed snapshot expiry is rejected.

Snapshot expiry is creation_time + max_command_age_s. Dispatch does not restart it; the solve deadline is the earlier of dispatch_time + solve_budget_s and snapshot expiry. The independent supervisor uses real monotonic time for execution budgets. Advancing a fake clock cannot establish or bypass wall-time isolation.

Input freshness, receipt liveness and snapshot expiry are separate records. A normally newer joystick sequence does not invalidate a bounded result by sequence equality alone. command_seq is local dispatch/local-hold bookkeeping, echoed by the worker; it is not added to ControlSnapshot or substituted for snapshot_seq. Older or forged sequences, old session/epoch results and duplicates cannot override a newer accepted decision or refresh its watchdog.

For equal test-clock timestamps, process release/disable/fault/invalidation first, then fresh input/state events, then eligible result acceptance, then target-expiry/liveness checks, then new dispatch. An acceptance may refresh liveness before the watchdog at the same tick, but cannot rescue a snapshot already at its own expiry. These event-order rules are deterministic test settings, not operating-system scheduling guarantees.

## 3. Deterministic intent support

Resolve the exact nonempty semantic candidate set before calculation. Each candidate has its existing relevance t_i in [0,1], at least one compatible program, and exactly one approach-keypoint mapping in the estimator configuration. The mapping uses IDs, never labels or array storage order. Missing mappings/programs, duplicate IDs, invalid numbers or unresolved references fail explicitly; do not drop a bad candidate or repair the result to uniform support.

For candidate i, form d_i by normalizing the XY vector from the snapshot's measured TCP to its mapped approach point. If its norm is at most 1e-12 m, d_i is zero. Let m = norm(u), with u the current normalized XY input. For enabled nonzero input, u_hat = u/m. Exact zero is neutral because dead-zone processing belongs to the calibrated input adapter.

The assembler supplies at most 32 earlier, time-ordered current-session samples within the 1 s window, excluding the current sample. Select the latest 32 before freezing the snapshot, preserving sequence order for equal timestamps. Referenced samples must resolve, use the same mapping_id and comparable clock domain, and have valid numeric input. Duplicate, future, out-of-window or oversized referenced history is invalid. Compute h as the arithmetic mean of unit directions of enabled, nonzero samples in that history; disabled/neutral samples contribute no direction. If none remain, h = (0,0). There is no hidden recursive estimator state.

$$
C_i=(1+\hat u^\top d_i)/2,\qquad H_i=(1+h^\top d_i)/2,
$$
$$
a_i=0.25+t_i+2C_i+0.5H_i,\qquad w_i=a_i/\sum_k a_k.
$$

Sort output by object_id for serialization while preserving ID semantics. The positive floor is part of the explicit formula for valid candidates; it does not repair missing data. Equal valid scores yield equal support. All-zero relevance is valid, so direction/history can still distinguish candidates. These are relative support values, not calibrated probabilities.

For active enabled nonzero input, assistance_strength = alpha_active = 0.5, independently of w. The alpha-zero configuration sets it to zero. Neutral/disabled input uses u_hat = (0,0) if a diagnostic weight record is formed, sets alpha to zero, and returns a disabled disposition: no motion solve. Invalid/no candidates produce no usable ObjectWeights even if input is neutral.

For opposite targets, the current-direction coefficient 2 exceeds the maximum adverse relevance difference 1 plus history difference 0.5. Complete reversal therefore changes the preferred opposite target immediately. Continuous geometric assistance may still reduce progress; the host half-space in section 4 prevents an opposing next step. A candidate exactly at the TCP is direction-neutral; it is not discarded.

## 4. Pose objective, smoothing and solver

Let p,R be measured TCP pose; R maps TCP to control coordinates, with TCP X as approach. Let L = max_linear_speed_mps * dt and A = max_angular_speed_radps * dt. In the reference session L = 0.01 m and A = 0.1 rad. Optimize six dimensionless values x,r:

$$
p'=p+Lx,\qquad R'=\operatorname{Exp}([Ar]_\times)R,\qquad
b=(\mathrm{max\_xy\_speed\_mps}\,dt/L)\,u.
$$

The rotation increment is in the control frame and left-multiplied. Emit the existing normalized xyzw quaternion, using a consistent nonnegative scalar component for these small increments; do not add Euler fields or directly add quaternions. Use principal SO(3) log for differences; the reference angular step is below pi.

Resolve previous_applied_command_id through the actual adapter application record, result and its source snapshot/state. For a move, define reference velocities v_prev = (p_target - p_source)/dt_prev and omega_prev = Log(R_target R_source^T)/dt_prev, then x_prev = v_prev*dt/L and r_prev = omega_prev*dt/A. These are accepted-command reference velocities, not measured motion. Null, a previous hold, or a legitimate same-session previous command from an old epoch gives zero reference. A nonnull missing, unaccepted or cross-session reference is an error, not zero history; never scan past a hold to revive an older move.

$$
J=\|x_{xy}-b\|^2+
0.1\left(\|x-x_{\mathrm{prev}}\|^2+\|r-r_{\mathrm{prev}}\|^2\right)
+\alpha\sum_iw_i\sum_{j\in\mathcal P_i}\lambda_j
\left[\max(0,g_j(p',R')/s_j)\right]^2.
$$

Every term is dimensionless. Translation in all three axes and rotation participate in smoothing; user following is XY only. s_j has the residual's unit; default scales are 0.01 m, 0.1 rad and 1 for dimensionless relations. Default lambda_j = 1. ProgramExecutionBinding pins each scale, weight and receipt. A change requires new binding/config IDs. Recompute every relation at each candidate; positive residuals are expected during approach.

Basic hard margins are m^2 - norm(x)^2, m^2 - norm(r)^2 and u_hat dot x_xy, plus the host margins in section 5. Thus input magnitude bounds both translational and angular assistance. Neutral/disabled input bypasses the solver and cancels motion through the existing local path.

Use scipy.optimize.minimize with method SLSQP, zero initial x,r, componentwise bounds [-1,1] for all six variables, jac=None, eps=1e-6, ftol=1e-9, maxiter=100, disp=False and no parallel finite differences. The input-scaled norm margins remain required in addition to these box bounds. [SciPy documents these parameters](https://docs.scipy.org/doc/scipy-1.11.4/reference/optimize.minimize-slsqp.html). Use zero initialization for every snapshot, not hidden warm-start state.

The non-opposition rule applies to each next proposal using pinned input and again to acceptance using current input/state. Plant interpolation preserves the rule relative to the input accepted for that bounded target. A later ordinary nonzero direction update affects the next gate check; it does not change epoch or require neutral re-engagement. This is not an instantaneous physical-reversal guarantee between control cycles. Current release/disable always overrides the remaining target immediately.

Success requires solver success, finite pose/objective/evaluations, completion before the real/local deadlines, and independent final recomputation of all hard margins. SLSQP's iteration limit is not a time bound. A resource/iteration/deadline budget exhaustion gives timeout; incompatible inequalities or a returned hard-infeasible point gives infeasible; other unsuccessful/nonfinite/exceptional calculations give error. All produce hold without pose/horizon; never emit an unfinished iterate or reuse a previous target. Failure of any required evaluator invalidates the whole solve, including when alpha is zero. constraints_satisfied describes all generated residuals at the final candidate, not hard feasibility or command acceptance.

## 5. Host constraints, synthetic controller and plant

### Shared callback boundary

The shared library accepts a pure checker of candidate pose, supplied measured state, u, dt and immutable constraint configuration. It returns named finite dimensionless margins, violation reasons and a feasible flag computed from those margins. It does not read clocks, current devices, mutable globals or network state. Solver calls use pinned data. The robot gate calls the same mathematical contract independently with fresh local data and rechecks authority, deadline, ordering and controller state outside the callback.

The algorithm supplies basic step margins from section 4; the injected checker supplies model, workspace and path margins. The gate independently recomputes both sets: derive x=(p_target-p_current)/L and r=Log(R_target R_current^T)/A using the current measured pose and the unchanged candidate horizon, then use current input magnitude/direction for all three basic margins. It also checks the six componentwise bounds. Do not trust margins reported by the worker. Configuration identities must agree, and a callback error/nonfinite margin is rejection. Each margin must be >= -1e-8; this is a numerical comparison tolerance, not a hardware safety margin. The consumer owns the implementation. Server unit checks use local doubles, without importing ROS2 source.

### Synthetic model and constraints

Define six named synthetic rotary channels q1 through q6, all in radians:

$$
o=(0.4,0,0.3)\ {\rm m},\quad s=(0.2,0.2,0.2)\ {\rm m},\quad
p=o+s\odot\sin(q_{1:3}),\quad R=R_z(q_6)R_y(q_5)R_x(q_4).
$$

All angles lie in [-pi/3,pi/3]. This is a nonphysical rotary-coordinate proxy, not LIO or a real-arm FK/IK model. Its length scale has units of metres; position is never stored as radian joint data. Position inverse is componentwise asin((p-o)/s). Orientation inverse is the unique ZYX branch q4=atan2(R32,R33), q5=asin(-R31), q6=atan2(R21,R11), using one-based matrix indices. Reject out-of-domain or out-of-bounds inverses; do not clip them into feasibility. The angle bounds avoid gimbal lock.

Additional hard constraints:

- AABB workspace: X [0.3,0.5], Y [-0.1,0.1], Z [0.2,0.4], all metres. Divide signed face margins by 0.2 m.
- Joint inverse bounds: each signed distance to +/-pi/3, divided by pi/3.
- An excluded sphere centered at (0.45,0.07,0.30) m, radius 0.02 m, plus 0.0001 m clearance. For the entire straight segment from supplied p to candidate p', compute closest distance d_seg; margin = (d_seg - 0.0201 m)/0.2 m. A zero-length segment uses point distance. Checking endpoints alone is insufficient.
- Finite normalized pose, compatible model/frames and agreement between supplied q and proxy_forward(q), within section 6 tolerances.

This tests an independently specified synthetic forbidden region, not real link/gripper collision checking. Positive generated residuals cannot relax any of these checks.

### Controller/plant behavior

Keep proposed results, gate acceptance, controller target and independent plant state as separate records. Initial state is q=0, hence p=o,R=I. apply installs only a newly accepted target, acceptance-time input magnitude/direction and a fixed target expiry. It does not set measured state to that target. An accepted newer target replaces the earlier one.

At each 0.02 s plant tick, while target authority is valid, compute dp = p_target-p and dtheta = Log(R_target R^T). Use

$$
\gamma=\min\left(0.5,\frac{v_{\max}m\,dt_{\rm plant}}{\|dp\|},
\frac{\omega_{\max}m\,dt_{\rm plant}}{\|d\theta\|}\right),
$$

omitting ratios whose denominator is zero. Set p_new=p+gamma*dp and R_new=Exp(gamma*dtheta)R, then inverse-map to q_new. This provides lag and bounded velocity rather than teleporting to a command. Validate inverse bounds, path and speed before committing q_new; an unexpected internal model violation is a controller fault with no state update. Joint-space half interpolation is not used because it need not preserve the Cartesian path.

Read RobotState from the committed independent q at the tick, with tcp_pose=proxy_forward(q), state_source=fk_from_measured_joints and matching sampled_at. Optional joint velocities are (q_new-q_old)/dt_plant. Every log/manifest labels these synthetic measurements; they do not establish real measured-state provenance.

cancel removes the target; hold uses current valid sampled state and stops advancement; stop clears the target and sets synthetic velocities to zero without needing fresh feedback. Expiry also clears the target. Release/disable triggers the existing coordinator withdrawal/epoch change and local cancel/hold or stop immediately, outside solver completion. A stale-feedback stop is a synthetic controller action, not proof of physical stopping. Controller fault/stop failure follows latched FAULT and explicit rearm. Ordinary release retains a valid bundle; invalid context requires reinitialization; computation/freshness failures enter PAUSED and require neutral then a new engagement. A discarded obsolete result never stops a newer accepted target.

## 6. Numerical policy and enforceable limits

All numerical evaluation uses float64. Literal indices remain integers; numeric expression literals have floating-point value. A return must be a finite scalar, not a Boolean or array. Preserve exact UTF-8 source bytes and SHA-256; program identity additionally covers bindings, parameters and declarations. Do not execute source with Python eval/exec.

| Policy item | Bound or comparison |
| --- | --- |
| Source / AST nodes / AST depth / assignments | 8192 UTF-8 bytes / 512 / 32 / 64 |
| Candidate objects / total programs per snapshot | 8 / 16 |
| Keypoint / axis / parameter bindings per program | 32 / 32 / 32 |
| Array intermediates | Rank <= 2, <= 256 float64 elements; only known fixed shapes and scalar-to-array arithmetic |
| Quaternion/unit-axis norm; rotation orthogonality and determinant | Absolute 1e-8; no silent normalization of arbitrary inputs |
| Weight sum; reproducible weight comparisons | Absolute 1e-12 |
| Raw residual satisfaction | g <= 1e-9 in its declared output unit |
| Replay residual/penalty comparison | atol=1e-9 in declared unit (penalty dimensionless), rtol=1e-9 |
| Analytical solver-position/rotation comparisons | 1e-6 m; SO(3) angular error 1e-5 rad |
| Synthetic state/model agreement | 1e-9 m and 1e-8 rad |
| Hard normalized margins | >= -1e-8 |
| Evaluator address space / solver address space | 256 MiB / 1 GiB, per process |
| Evaluator call / total solve real-wall deadline | 0.01 s / 0.15 s, additionally bounded by original snapshot expiry |

Reject source size before parsing; count/depth-check the parsed AST before interpretation. Enforce the existing straight-line grammar and primitive signatures, exact binding shapes and all array operation/result bounds. Every intermediate must remain finite. Mathematical domain/division errors are failures. An artifact's units and feature meaning must be checked against its declarations and the fixture geometry; probes cannot prove arbitrary generated semantics correct.

Use separately spawned, supervised evaluator and solver processes. The evaluator is a standard-library restricted numerical interpreter; the solver uses NumPy/SciPy. Set BLAS thread counts to one before imports. Apply Linux resource.RLIMIT_AS with equal soft/hard limits, disable core dumps, and verify the configured limits before accepting work. [Python's resource API defines RLIMIT_AS as address-space size](https://docs.python.org/3.12/library/resource.html), not a resident-memory measurement. Startup/import failure under these bounds fails preparation; do not silently raise limits.

Send bounded validated numerical data through per-worker IPC, with a 1 MiB payload ceiling. Keep pipe waits and blocking reads out of input/release/gate callbacks. The supervisor enforces real monotonic deadlines, withdraws current computation on timeout and discards its output, then terminates and, after at most 0.1 s teardown grace, kills the worker if needed. Teardown grace is not extra motion authority or solve time. Retire its IPC rather than reusing a possibly corrupted queue. Restart/prepare workers while motion is disabled. Generation programs cannot access files, network, host objects or subprocess creation through the interpreter. Process separation and grammar checks are distinct protections, not a claim of a general-purpose hostile-Python sandbox.

Versioned probe set for each program/binding:

1. Resolve the fixture's exact features and parameters. Probe p=o and o +/- L*e_k for each of the three axes, crossed with R=I, Exp(+A*[e_k]x) and Exp(-A*[e_k]x). This is 49 valid finite pose probes per fixture. Preserve probe order X,Y,Z, positive before negative.
2. Add each bound approach target with R=I, the directed-axis aligned and opposed TCP-X orientations, and zero feature/parameter arrays only for programs with legitimately empty bindings. All required outputs must be finite scalars.
3. Reverse each undirected axis individually, preserving its identity; residuals must agree within replay tolerance. Directed-axis opposition need not agree. Reorder scene storage while retaining IDs; resolved inputs/results must agree.
4. Reject malformed rotations, zero axes, wrong shapes, unresolved/foreign IDs, changed source/hash/content, out-of-range indices, forbidden syntax, divide-by-zero/domain/nonfinite output, oversized source/AST/arrays and resource exhaustion. These rejection fixtures are separate from valid probe results.

For aligned/opposed probe orientations, map TCP X to the requested unit direction by the shortest rotation with axis proportional to eX cross direction. Choose I for parallel directions and Rz(pi) for antiparallel directions. These evaluator probes need not be feasible under synthetic motion/joint bounds.

Parameter values are not randomly perturbed. Each probe is reproducible from numeric-offline-v1 and the named immutable fixture. When features, policy or program meaning change, issue the required new validation receipt; no acceptance is inferred merely from a matching source hash.

## 7. Worked fixtures and retained calculation evidence

All numerical examples use p=o,R=I, enabled u of unit magnitude, dt=0.2 s, L=0.01 m,A=0.1 rad, lambda=1, and previous command reference zero unless stated. The fixed scene has bottle/cup IDs with distinct approach-keypoint IDs. The default reversal fixture places their targets at o+(L,0,0) and o-(L,0,0), relevance 0.5 each, distance tolerance zero and scale L. History consists of one earlier rightward sample at 9.98 s, with current input and independent state at 10 s. A previous applied rightward reference, where specified, has velocity 0.05 m/s, reconstructed from its own source and target over 0.2 s.

The distance program is the existing norm(p_tcp-keypoints[0])-params[0], with a declared metre tolerance. The axis program is acos(clip(dot(R_control_tcp[:,0],axis_dirs[0]),-1,1))-params[0], with a radian tolerance; axes used by these alignment fixtures are directed. These use the existing language primitives and bind by ID.

| Named fixture | Inputs or event | Expected oracle |
| --- | --- | --- |
| joystick-only | Alpha-zero configuration; u=(1,0), valid default programs, zero prior reference | delta_x=10L/11=0.0090909090909 m, other increments zero; solved proposal, subject to gate checks |
| positive-approach | Single bottle target o+(2L,0,0), tolerance L, scale L, alpha=0.5, u=(1,0) | delta_x=15L/16=0.009375 m; g=L/16=0.000625 m; penalty=1/256; evaluation_valid=true, solved, constraints_satisfied=false; gate may accept |
| axis-alignment | Single candidate; only the axis program, tolerance 0, scale A; target axis=(cos A,sin A,0); u=(1,0) | rotation increment Z=5A/6=0.0833333333333 rad; residual A/6; translation X=10L/11 because current input remains in the objective |
| reversal | Default two targets; old rightward history and applied reference; current u=(-1,0) | left/right support=11/16,5/16; delta_x=-87L/128=-0.006796875 m; other increments zero |
| release | At t=10.04 s set u=(0,0) or enabled=false while a t=10 s solve is pending | Local cancellation, epoch increments once for the withdrawal, PAUSED, no further plant advancement; late prior-epoch result rejected without a target; fresh neutral/engagement needed to resume |
| host-rejection | Current p=(0.499,0,0.3) m, u=(1,0); received target=(0.504,0,0.3) m with otherwise valid identity/timing | Step bound passes but workspace margin=-0.004/0.2=-0.02; command_accepted=false, applied_command_id=null; candidate never reaches plant, current-authority rejection uses local hold |
| computation-error | Required accepted program encounters division by zero at its current arguments; alternatively stall past 0.15 s while expiry remains 0.2 s | Hold with no pose/horizon; error/evaluation_valid=false for invalid evaluation, timeout for stall; never reuse partial or previous target; coordinator follows PAUSED recovery |

The division-error variant uses a dimensionless program returning params[0] / (p_tcp[0] - params[1]), with parameters 0.01 m and 0.405 m, no feature bindings, scale 1 and the default scene targets. It is finite at the named validation poses/targets but fails when the runtime measured X is 0.405 m. This illustrates why accepted probes do not certify every future evaluation. The stall variant is injected into the worker boundary; looping generated source remains forbidden.

With no history and current right input, the default two-target supports are right=3/4,left=1/4. Under reversal with maximally adverse relevance (old right=1,new left=0) and rightward history, left still receives 9/16 versus right 7/16. Candidate storage order cannot change these values. Invalid/no candidates fail; valid equal candidates can be equal. Neutral/disabled input cannot be rescued by old directional history.

For synthetic feedback, apply the positive-approach target at t=10 s. On the first 0.02 s plant tick, speed limiting sets gamma=0.001/0.009375; actual X advances 0.001 m, not 0.009375 m. The sampled q1 is asin(0.001/0.2), with TCP reconstructed at o+(0.001,0,0); proposed, accepted and observed motion differ explicitly.

The position, rotation and weight comparison tolerances were fixed before numerical execution. A standalone [calculation script](https://github.com/ZenWang00/semantic-shared-control/blob/main/docs/reference/verify_offline_profile.py) and its [retained JSONL output](https://github.com/ZenWang00/semantic-shared-control/blob/main/docs/reference/offline-profile-oracles.jsonl) accompany this profile. Run from the repository root in the inspected environment:

~~~sh
python3 -B docs/reference/verify_offline_profile.py
~~~

On 25 October 2026 the five support checks and four six-dimensional SLSQP cases passed on the environment in section 1. Maximum observed position error was approximately 3.04e-8 m and rotation error 6.81e-7 rad, both below their predeclared thresholds. The script uses analytical fixture formulas directly; it is not the future restricted interpreter, snapshot resolver, optimizer pipeline or gate. Iteration counts are diagnostics, not latency measurements. Release, host-rejection, computation-error and plant rows are specified design oracles, not implemented-runtime test results.

## 8. Acceptance boundary and downstream handoff

Downstream cards implement this profile with the identities above and retain evidence against the existing S/P/G/C matrix. Implementers must not silently substitute a different estimator, solver, platform budget or model because an example times out. Record a new profile/configuration revision and the reason if limits require adjustment.

P05 capture-time-transform adapter behavior remains deferred: the 0.01 s capture-skew setting and coherent metadata are not adapter evidence. P08 evidence is limited to independently advanced synthetic feedback; no physical joint/TCP synchronization or real stopping behavior is established. Package isolation, process memory/time enforcement, controller lifecycle, complete-loop integration, archive round-trip and two-repository compatibility still require their downstream execution cards.

This card is accepted through complete design coverage, dimensional/contract inspection, reproducible numerical design checks, independent review of the final commit and Wiki publication from the merged source. It makes no thesis claim of improved intent inference, task performance, hardware safety or real-time capability.
