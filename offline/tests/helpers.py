"""Real candidate selection and fixed metric for lifecycle-only worker doubles."""
import math
from ssc_core import NUMERIC_CANDIDATES_V2, estimate_intent, get_intent_config, to_dict


def selection_and_evaluation(source, selection_id, position):
    selected = estimate_intent(source.resolved, get_intent_config("intent-candidates-v2"),
                              selection_id=selection_id, policy=NUMERIC_CANDIDATES_V2).selection
    point = next(p for p in source.bundle.resolved.candidates if p.candidate_id == selected.selected_candidate_id)
    distance = math.dist(position, point.position_xyz)
    return to_dict(selected), dict(candidate_id=point.candidate_id, distance_m=distance, raw_g=distance,
                                  normalization_scale=.01, penalty=(distance / .01) ** 2)
