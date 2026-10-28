# Offline Candidate Position Profile

The schema-2 candidate profile replaces the generated-program profiles. All settings below are synthetic experiment settings, not measured robot limits or timing guarantees.

| Setting | Value |
| --- | --- |
| Control / TCP frame | synthetic_base / synthetic_tcp |
| Control horizon | 0.2 s |
| Max XY / full linear speed | 0.05 / 0.05 m/s |
| Max angular speed for independent host checks | 0.5 rad/s; proposed orientation is held |
| Step normalization L | 0.01 m |
| Assistance alpha / smoothing | 0.5 / 0.1 |
| Distance scale / tolerance | 0.01 m / 0 m |
| Selector switch margin | 0.1 raw score, strict greater-than |
| Maximum candidates | 16 |
| Input/state source age limit | 0.5 s |
| Snapshot expiry / solve budget | 0.2 / 0.15 s |
| Plant tick | 0.02 s |
| History | At most 32 earlier samples within 1 s |

For dimensionless translation `x`, `p_next=p_measured+L*x`. The fixed objective is XY input tracking plus `0.1*||x-x_prev||²` plus `alpha*max(0,(distance(p_next,k_selected)-tolerance)/scale)²`. Basic constraints bound the full step by current input magnitude, require a nonnegative XY projection onto current input, and impose independent host margins. SLSQP uses zero initialization, eps=1e-6, ftol=1e-9 and maxiter=100. A failed or expired solve yields no motion target.

The quaternion remains equivalent to the snapshot's measured quaternion. There are no angular optimization variables or angular smoothing commands. Alpha-zero fixes Z to zero, even after a previous assisted vertical step. Joystick release cancels motion through the local path rather than asking the optimizer for a new target.

The synthetic rotary proxy retains `p=(.4,0,.3)+.2*sin(q[0:3])` and its original orientation mapping, workspace/path constraints and independent speed-limited feedback. It is not a physical robot FK/IK model. New fixtures include higher/equal/lower targets, same-object alternatives and 1/2/16 candidates. Results must distinguish proposed target, accepted command and observed state.

Position oracle tolerance is 1e-6 m; held-orientation comparison uses rotation equivalence, not raw quaternion sign. Fixed-distance replay uses atol=rtol=1e-9. The installed package versions, Git revisions and wheel hashes accompany every acceptance run.
