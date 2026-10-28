# Atomic candidate context activation

`ssc-runtime==0.8.0` uses schema 2.0 candidate data. Shared contracts remain in the
[canonical server documentation](https://github.com/ZenWang00/semantic-shared-control/blob/main/docs/wiki/Data-Model-and-Message-Contracts.md).
Activation is runtime state, not a wire message or motion permission.

Construct one `ActiveBundleStore(session_id)` per session. The host supplies
`HostCompatibility`; defaults pin `synthetic_base`, `synthetic_tcp`,
`synthetic-rotary-v1` and `synthetic-host-v1`. These installed identities are
compared against configuration independently of candidate content.

1. With motion disabled, `with store.artifacts() as registry` registers immutable
   configuration, task, observation, geometry, semantics and `CandidateSet`.
2. `ActivationRequest` identifies the session, configuration, task revision,
   scene, observation, geometry revision, feature set, semantics and candidate set.
   `begin(request)` immediately removes the active reference and supersedes older
   attempts, including equal-valued requests.
3. `prepare(attempt)` resolves the full data graph and checks host compatibility.
   Candidates must reference the exact task/scene/feature/observation/control frame.
   Each candidate is a finite TCP position with a unique stable ID; 1–16 are allowed.
   Failure returns a diagnostic and no old-context fallback.
4. `publish(prepared)` accepts only this store's exact pending preparation object.
   Copied, foreign, consumed or superseded results cannot publish. It swaps one
   complete immutable bundle; fresh engagement and feedback are still required.
5. `invalidate(reason)` removes active context and pending attempts immediately.
   `acquire()` and `is_current(bundle)` are instantaneous observations, not leases.

The artifact lock is separate from state publication. Invalidation never waits
for artifact resolution; a late preparation cannot restore superseded context.
Duplicate concurrent preparation is rejected without canceling the first attempt.
Unexpected host errors remain visible and retire only the still-current attempt.

`ActiveBundle.resolved` contains configuration, task, geometry, semantics and the
candidate set. `observation` and `semantic_observation` retain capture provenance;
semantics can reference an earlier capture of the same static scene. Candidate
positions are pinned to the geometry observation. Mutable input dictionaries cannot
alter either a published bundle or retained old snapshots.

Lifecycle must withdraw motion authority, advance the epoch and request cancel plus
hold/stop before replacing context. Registration alone never activates new targets.
Snapshots retain old data for evidence while the gate rejects its use for motion.
Tests cover complete and incompatible context, candidate replacement, detached
records, concurrent readers, invalidation during resolution and stale publication.
