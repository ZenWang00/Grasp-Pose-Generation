# Atomic context activation

This implementation of P1-08 is internal runtime state, not a public wire
message or motion permission. Shared contracts and policy remain in the
[canonical server Wiki source](https://github.com/ZenWang00/semantic-shared-control/tree/95d618f567f5f057a1b3f1737cc394a105fa9a4e/docs/wiki).

## Host interface

Construct one `ssc_runtime.ActiveBundleStore` per session from trusted
`UnitReview` and `HostBindingPolicy` tuples. It owns one `SessionRegistry` and
the matching `HostProgramValidator`; a second store cannot restore local trust
from archived receipts. The default `HostCompatibility` pins
`synthetic_base`, `synthetic_tcp`, `synthetic-rotary-v1` and `synthetic-host-v1`.
These are installed host identities, independently compared to SessionConfig.
Other compatibility values require an explicitly configured host, not values
copied from generated artifacts. Numeric policy is exactly numeric-offline-v1.

1. With motion disabled, use `with store.artifacts() as (registry, validator)`
   to register immutable inputs and issue real receipts and host bindings using
   `validator.validate(..., motion_disabled=True)` and `create_binding(...)`.
   Keep all accesses to these mutable services inside this context, including
   future downstream resolution. Do not retain/use them unlocked or wait for
   another preparation thread while holding this lock.
2. The local coordinator forms an `ActivationRequest` with exact session,
   configuration, task revision, scene, geometry observation/revision, feature
   set, semantic ID, program hashes and binding IDs. It calls `begin(request)`
   before preparing that requested context. This immediately removes the active
   reference and supersedes earlier attempts, including equal-valued requests.
3. `prepare(attempt)` resolves the complete immutable graph, checks host
   compatibility and requested identities, and verifies every receipt/binding
   through the owning validator. Its outcome is `prepared`, `rejected` with a
   diagnostic code/path, or `superseded`. It does not publish context.
4. `publish(prepared)` accepts only this store's exact, pending preparation
   object. A consumed, copied, forged, rejected or superseded result cannot be
   published. The state lock swaps one complete `ActiveBundle` reference and
   returns `activated`/`context_ready`. No joystick/feedback or controller API
   is called, and no epoch or motion-enable flag is changed here.
5. `acquire()` returns the current immutable bundle or `None`. Retained bundles
   preserve their old content. `is_current(bundle)` checks identity at that
   instant; it is not a lease across a subsequent invalidation. `invalidate(reason)`
   removes the active reference and pending attempts immediately and returns an
   explicit invalidation outcome. Recovery starts with a new attempt.

Registry and authority checks run under a separate artifact lock. The state
lock is never held while waiting for it. Publication, begin, invalidation and
read acquisition share one state lock. Therefore a slow preparation cannot
prevent invalidation; its final currency check cannot clear or replace newer
state. Duplicate concurrent preparation is rejected with `attempt_busy` without
canceling the original preparation. An unexpected host exception is propagated
and retires only that still-current attempt. There is no automatic fallback.

`ActiveBundle.resolved` contains the config, task, geometry, semantics,
candidates, ordered programs/feature arrays, accepted receipts and execution
bindings. `observation` and `semantic_observation` preserve both capture records:
the shared contract permits semantic provenance from an earlier observation of
the same static scene. Both are resolved and frame/scene checked; their IDs need
not be equal. Public records are reparsed into immutable records/tuples by the
shared library; mutable input dictionaries cannot alter a published bundle.

## Integration obligations

[Snapshot assembly (#2)](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/2)
must acquire once and pin that exact context, then associate its activation
identity with local dispatch records. It must separately validate feedback,
joystick/history and local timing. A bundle reference does not authorize dispatch
or make an old command current.

[Lifecycle (#4)](https://github.com/ZenWang00/Grasp-Pose-Generation/issues/4)
must withdraw motion authority, advance its cancellation epoch, cancel work and
request hold/stop before replacement/invalidation. A coordinate-only geometry
revision follows this same paused begin/prepare/publish path, although valid
receipts may be reused for unchanged feature definitions. Successful activation
only allows context readiness; fresh eligible engagement and measured feedback
are still mandatory. Session restart creates a new store with a new session ID
and revalidates programs. Ordinary release does not itself invalidate a still
valid bundle. Final command acceptance must recheck context and local authority
as one coordinated decision, not rely on a prior `is_current()` result.

## Verification scope

The copied `offline/tests/fixtures/bottle_cup_offline_v1.json` is byte-identical
to the server fixture at the pinned revision. Its unissued binding placeholders
are never trusted: test setup creates fresh host receipts and bindings for both
bottle and cup. Fixture reviews describe each metre displacement norm minus a
metre tolerance; they are trusted test inputs, not generated unit evidence.

| Acceptance/scenario | Evidence |
| --- | --- |
| Exact graph and host compatibility | complete-bundle, host-frame/model, semantic mismatch and missing-reference tests |
| Local receipt trust and normalization | imported/rejected/mismatched receipts, archive restart and host-normalization tests |
| P01–P04 activation obligations | semantic task/scene/features mismatch; late attempt/task/scene/config results; atomic reader interleavings; retained old references |
| Coordinate replacement and activation portion of P07 | paused revision replacement, reused valid receipt with new geometry, explicit invalidation and no fallback |
| Real producer/consumer boundary | Linux test validates both programs in bounded workers, verifies cleanup, creates bindings, prepares and publishes a complete bundle |

Concurrency tests use events and futures with bounded waits. Readers are
checked before/during/after publication, and an old preparation is deliberately
completed after a newer activation. The test writes `concurrency-trace.json`
under pytest's retained `--basetemp` directory. CI archives it with JUnit, wheels,
exact source revisions and dependency versions. Most tests stub only numerical
probe success to isolate activation behavior; the Linux test exercises actual
worker validation. These are software component checks, not the full P01–P04/P07
integration acceptance, capture-time adapters, motion safety or robot evidence.
