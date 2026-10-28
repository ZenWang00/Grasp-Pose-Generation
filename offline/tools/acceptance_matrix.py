"""Join executed JUnit cases into candidate-data, selection, numerical and runtime obligations; fail on missing evidence."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

# Every listed selector is mandatory. All parametrizations must pass. Component
# tests remain component evidence; the integration column names full-loop tests.
ROWS = {'D01': [('server', 'test_all_catalog_messages_round_trip_and_are_detached'),
         ('server', 'test_reject_invalid_candidate_data'),
         ('server', 'test_reject_old_wire_version_and_program_types')],
 'D02': [('server', 'test_fixed_vlm_response_is_data_only_with_order_independent_host_ids'),
         ('server', 'test_reject_invalid_vlm_proposals'),
         ('server', 'test_parser_rejects_duplicate_keys_and_no_implicit_grounding')],
 'D03': [('runtime', 'test_all_shipped_scenarios_activate_without_solver'),
         ('runtime', 'test_candidate_count_scenarios_activate_without_solver'),
         ('runtime', 'test_incompatible_candidate_context'), ('server', 'test_reject_candidate_context_mismatch')],
 'S01': [('server', 'test_direction_selects_one_real_candidate_and_replays'),
         ('server', 'test_input_order_cannot_change_selection'),
         ('server', 'test_same_object_multiple_candidates_tie_distance_then_id'),
         ('runtime', 'test_same_object_alternatives_select_one_target')],
 'S02': [('server', 'test_switch_requires_strictly_more_than_point_one'),
         ('runtime', 'test_current_direction_selection_and_accepted_smoothing')],
 'S03': [('server', 'test_previous_selection_pins_earlier_same_context'),
         ('runtime', 'test_selection_history_is_explicit_and_cleared_on_release_epoch_and_context')],
 'N01': [('server', 'test_selected_target_analytic_oracle'),
         ('server', 'test_objective_is_fixed_unweighted_distance'),
         ('runtime', 'test_profile_golden_movement')],
 'N02': [('server', 'test_same_xy_input_supplements_height_from_target'),
         ('runtime', 'test_target_height_assistance_preserves_measured_orientation')],
 'N03': [('server', 'test_previous_reference_uses_translation_and_ignores_angular_history'),
         ('server', 'test_nonidentity_orientation_is_copied_including_quaternion_sign'),
         ('runtime', 'test_gate_preserves_nonidentity_source_orientation_with_quaternion_sign_equivalence'),
         ('runtime', 'test_gate_rejects_rotation_even_when_original_step_limits_allow_it')],
 'N04': [('runtime', 'test_gate_rejects_z_motion_when_assistance_is_disabled'),
         ('server', 'test_alpha0_drops_previous_z_and_rotational_motion'),
         ('server', 'test_no_assistance_has_exact_zero_z_even_with_previous_z'),
         ('server', 'test_target_does_not_latch_and_input_can_move_past_it')],
 'R01': [('runtime', 'test_concurrent_completion_and_atomic_readers'),
         ('runtime', 'test_late_prepared_result_cannot_restore_old_context'),
         ('runtime', 'test_registered_candidates_cannot_mutate_active_bundle')],
 'R02': [('runtime', 'test_stale_inputs_stop_active_target'),
         ('runtime', 'test_foreign_production_clock_is_not_subtracted'),
         ('runtime', 'test_missing_samples_do_not_become_zero')],
 'R03': [('runtime', 'test_expired_result_cannot_extend_snapshot_lifetime'),
         ('server', 'test_caller_deadline_only_shortens_budget')],
 'R04': [('runtime', 'test_duplicate_forged_and_old_results_cannot_cancel_new_target'),
         ('runtime', 'test_new_loop_has_fresh_session_and_rejects_prior_session_work')],
 'R05': [('runtime', 'test_worker_stall_times_out_without_renewing_target'),
         ('runtime', 'test_worker_stall_keeps_release_local'),
         ('runtime', 'test_worker_death_holds_and_requires_repreparation'),
         ('server', 'test_repeated_failures_do_not_leak_process_groups')],
 'R06': [('runtime', 'test_host_rejection_never_applies_proposal'),
         ('runtime', 'test_soft_relation_cannot_relax_real_host_limits'),
         ('runtime', 'test_entire_segment_not_only_endpoints')],
 'R07': [('runtime', 'test_neutral_step_cancels_before_plant_advancement'),
         ('runtime', 'test_release_inflight_and_ordinary_reengagement')],
 'R08': [('runtime', 'test_normal_loop_history_and_independent_feedback'),
         ('runtime', 'test_receipt_does_not_manufacture_feedback_and_first_tick_matches_profile')],
 'R09': [('runtime', 'test_context_reinitialization_and_explicit_fault_rearm'),
         ('runtime', 'test_stale_or_invalid_feedback_selects_stop')],
 'A01': [('runtime', 'test_real_run_archive_and_fresh_process_replay'),
         ('server', 'test_replay_recomputes_selection_switch_release_and_distances')],
 'A02': [('server', 'test_checksum_and_schema_are_enforced'),
         ('server', 'test_missing_record_rejected'),
         ('server', 'test_distorted_fixed_distance_is_detected_by_replay')],
 'P01': [('server', 'test_import_is_standalone_and_standard_library_only'),
         ('server', 'test_candidate_count_is_not_generated_program_count'),
         ('runtime', 'test_real_run_archive_and_fresh_process_replay')],
 'P05': []}


def generate(reports):
    cases = {role: list(ET.parse(path).iter("testcase")) for role, path in reports.items()}
    rows = []
    for identifier, selectors in ROWS.items():
        row = {"id": identifier, "status": "passed", "level": "software structural/semantic and simulated control; no hardware", "evidence": []}
        if identifier == "P05":
            row.update(status="deferred", level="adapter/hardware obligation",
                limitation="Capture-associated TF under arm motion requires the future capture adapter. Metadata tests do not satisfy P05.")
        if identifier == "R08":
            row["limitation"] = "Synthetic joint-derived feedback only; real measured-state provenance and hardware FK/TF comparison remain deferred."
        for role, selector in selectors:
            selected = [case for case in cases[role] if case.get("name").split("[")[0] == selector]
            passed = bool(selected) and all(not any(c.tag in ("failure", "error", "skipped") for c in case) for case in selected)
            row["evidence"].append({"report": str(reports[role]), "selector": selector,
                "cases": [{"name": c.get("classname") + "::" + c.get("name"), "seconds": c.get("time")} for c in selected],
                "passed": passed})
            if not passed:
                row["status"] = "failed"
        rows.append(row)
    # Unmapped failures also prevent aggregate acceptance.
    errors = [f"{role}:{c.get('classname')}::{c.get('name')}" for role, values in cases.items()
              for c in values if any(child.tag in ("failure", "error") for child in c)]
    return {"rows": rows, "unresolved_test_failures": errors,
            "passed": not errors and all(r["status"] in ("passed", "deferred") for r in rows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = generate({"server": args.server, "runtime": args.runtime})
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({"passed": report["passed"], "rows": len(report["rows"]),
        "failed": [r["id"] for r in report["rows"] if r["status"] == "failed"],
        "unresolved_test_failures": report["unresolved_test_failures"]}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
