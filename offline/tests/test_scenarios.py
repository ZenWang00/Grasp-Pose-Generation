"""Every shipped scenario must activate before a supervised solver is needed."""
import pytest
from ssc_runtime.loop import OfflineLoop
from ssc_runtime.scenarios import Fixture, VARIANTS


@pytest.mark.parametrize("variant", VARIANTS)
def test_all_shipped_scenarios_activate_without_solver(variant):
    with OfflineLoop(variant) as loop:
        loop.feedback()
        loop.input((0., 0.))
        outcome = loop.lifecycle.initialize(loop.request)
        assert outcome.status == "activated", outcome
        candidate_owners = {p.object_id for p in outcome.bundle.resolved.candidates}
        assert {r.object_id for r in outcome.bundle.resolved.semantic.object_relevance} == candidate_owners
        assert not loop.lifecycle.computation_ready


@pytest.mark.parametrize("count", [1, 2, 16])
def test_candidate_count_scenarios_activate_without_solver(count):
    fixture = Fixture(candidate_count=count)
    store, request = fixture.create_store("candidate-count-session")
    outcome = store.publish(store.prepare(store.begin(request)))
    assert outcome.status == "activated", outcome
    assert len(outcome.bundle.resolved.candidates) == count
    assert len({p.candidate_id for p in outcome.bundle.resolved.candidates}) == count
