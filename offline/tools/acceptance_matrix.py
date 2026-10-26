"""Join executed JUnit cases into all 34 S/P/G/C obligations; fail on missing evidence."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

# Every listed selector is mandatory. All parametrizations must pass. Component
# tests remain component evidence; the integration column names full-loop tests.
ROWS = {
    "S01": [("server", "test_complete_illustrative_catalog_parses_including_both_result_branches")],
    "S02": [("server", "test_required_fields_are_not_replaced_or_defaulted"), ("server", "test_structural_and_numeric_rejections_have_field_diagnostics")],
    "S03": [("server", "test_structural_and_numeric_rejections_have_field_diagnostics"), ("server", "test_joint_measurement_arrays_must_match_name_count")],
    "S04": [("server", "test_structural_and_numeric_rejections_have_field_diagnostics")],
    "S05": [("server", "test_internal_nonfinite_numbers_fail_before_use"), ("server", "test_nonstandard_json_numbers_are_rejected_at_load"), ("server", "test_live_nonfinite_numbers_never_reach_consumers")],
    "S06": [("server", "test_live_bad_types_domains_and_dimensions_have_precise_paths"), ("server", "test_joystick_unit_disk_does_not_borrow_quaternion_tolerance")],
    "S07": [("server", "test_object_weights_reject_empty_duplicate_and_negative_support"), ("server", "test_weights_sum_uses_absolute_reference_tolerance_without_normalizing"), ("server", "test_invalid_records_and_snapshot_references_never_return_weights")],
    "S08": [("server", "test_move_cannot_be_constructed_from_failed_or_targetless_result"), ("server", "test_hold_forbids_motion_fields_even_when_null")],
    "P01": [("runtime", "test_missing_program_feature_identity"), ("runtime", "test_receipt_and_binding_boundaries")],
    "P02": [("runtime", "test_late_prepared_result_cannot_restore_old_context"), ("runtime", "test_incompatible_semantic_context")],
    "P03": [("runtime", "test_incompatible_semantic_context"), ("runtime", "test_context_reinitialization_and_explicit_fault_rearm")],
    "P04": [("runtime", "test_control_frame_and_observation_mismatch"), ("runtime", "test_incompatible_state_blocks_snapshot")],
    "P05": [],
    "P06": [("runtime", "test_foreign_production_clock_is_not_subtracted"), ("runtime", "test_expired_result_cannot_extend_snapshot_lifetime")],
    "P07": [("runtime", "test_dispatched_arrays_survive_actual_geometry_revision"), ("runtime", "test_old_context_and_epoch_retain_evidence_but_cannot_dispatch")],
    "P08": [("runtime", "test_bad_mapping_and_commanded_feedback_are_rejected"), ("runtime", "test_normal_loop_history_and_independent_feedback")],
    "P09": [("runtime", "test_replayed_older_source_sequence_changes_nothing"), ("runtime", "test_session_restart_withdraws_before_clearing_and_old_session_cannot_revive"), ("runtime", "test_duplicate_forged_and_old_results_cannot_cancel_new_target")],
    "P10": [("runtime", "test_stale_inputs_stop_active_target"), ("runtime", "test_source_age_equality_is_fresh_and_stale_state_withdraws_on_input_path"), ("runtime", "test_stale_joystick_withdraws_and_holds_at_fresh_feedback"), ("runtime", "test_missing_samples_do_not_become_zero")],
    "G01": [("runtime", "test_profile_golden_movement")],
    "G02": [("runtime", "test_profile_golden_movement")],
    "G03": [("runtime", "test_wrong_host_normalization_and_missing_binding"), ("server", "test_no_review_wrong_units_policy_and_missing_settings")],
    "G04": [("server", "test_rejected_language"), ("server", "test_generated_source_cannot_reach_host_objects"), ("server", "test_compile_limits_enforced_inside_worker")],
    "G05": [("server", "test_full_artifact_identity_even_with_same_source_hash"), ("runtime", "test_receipt_and_binding_boundaries")],
    "G06": [("runtime", "test_computation_error_is_a_hold_and_requires_repreparation"), ("server", "test_probe_failure_retires_worker")],
    "G07": [("runtime", "test_worker_stall_times_out_without_renewing_target"), ("server", "test_infeasible_start_under_profile_budget_is_a_hold_without_target")],
    "G08": [("runtime", "test_real_run_archive_and_fresh_process_replay")],
    "C01": [("runtime", "test_normal_loop_history_and_independent_feedback"), ("runtime", "test_profile_golden_movement")],
    "C02": [("runtime", "test_neutral_step_cancels_before_plant_advancement"), ("runtime", "test_release_inflight_and_ordinary_reengagement"), ("runtime", "test_worker_stall_keeps_release_local")],
    "C03": [("runtime", "test_expired_result_cannot_extend_snapshot_lifetime")],
    "C04": [("runtime", "test_duplicate_forged_and_old_results_cannot_cancel_new_target")],
    "C05": [("runtime", "test_worker_stall_times_out_without_renewing_target"), ("runtime", "test_lost_upstream_progress_liveness_watchdog_and_discards_do_not_refresh")],
    "C06": [("runtime", "test_stale_or_invalid_feedback_selects_stop"), ("runtime", "test_cancel_hold_stop_prevent_later_advancement_without_copying_pose")],
    "C07": [("runtime", "test_release_inflight_and_ordinary_reengagement"), ("runtime", "test_context_reinitialization_and_explicit_fault_rearm")],
    "C08": [("runtime", "test_soft_relation_cannot_relax_real_host_limits"), ("runtime", "test_host_rejection_never_applies_proposal"), ("runtime", "test_entire_segment_not_only_endpoints")],
}


def generate(reports):
    cases = {role: list(ET.parse(path).iter("testcase")) for role, path in reports.items()}
    rows = []
    for identifier, selectors in ROWS.items():
        row = {"id": identifier, "status": "passed", "level": "software structural/semantic and simulated control; no hardware", "evidence": []}
        if identifier == "P05":
            row.update(status="deferred", level="adapter/hardware obligation",
                limitation="Capture-associated TF under arm motion requires the future capture adapter. Metadata tests do not satisfy P05.")
        if identifier == "P08":
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
