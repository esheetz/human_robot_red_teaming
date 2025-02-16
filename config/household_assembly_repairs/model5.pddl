(define (domain household_assembly_repairs)
  (:requirements :strips)

  (:predicates
    (furniture_unassembled)
    (furniture_partially_assembled)
    (furniture_assembled)
    (repair_detected)
    (repair_in_progress)
    (repair_completed)
    (repair_verified)
    (repair_failed)
    (tools_acquired)
    (tools_checked)
    (tools_incorrectly_used)
    (tools_stowed)
    (hazard_detected)
    (fire_hazard_detected)
    (electrical_hazard_detected)
    (preventative_maintenance_needed)
    (preventative_maintenance_completed)
    (emergency_repair_needed)
    (emergency_repair_in_progress)
    (emergency_repair_completed)
    (emergency_repair_failed)
    (cleaning_required)
    (cleaning_completed)
    (diagnostics_needed)
    (diagnostics_completed)
    (safety_check_passed)
    (human_intervention_required)
    (human_feedback_received)
    (inventory_checked)
    (inventory_low)
    (maintenance_required)
    (maintenance_completed)
    (human_notified_of_hazard)
    (human_notified_of_fire_hazard)
    (human_notified_of_electrical_hazard)
    (human_supervision_requested)
    (human_override_requested)
    (human_nearby_detected)
    (human_moved_from_area)
    (pet_or_child_supervision_requested)
    (task_prioritized)
    (self_maintenance_needed)
    (self_maintenance_completed)
    (environmental_constraint_detected)
    (low_battery_detected)
    (battery_recharged)
    (failure_log_updated)
  )

  (:action assemble_furniture
    :precondition (and (furniture_unassembled) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)) (not (human_nearby_detected)))
    :effect (and (furniture_partially_assembled) (not (furniture_unassembled)))
  )

  (:action complete_assembly
    :precondition (and (furniture_partially_assembled) (safety_check_passed) (not (environmental_constraint_detected)) (not (human_nearby_detected)))
    :effect (and (furniture_assembled) (not (furniture_partially_assembled)))
  )

  (:action detect_repair
    :precondition ()
    :effect (repair_detected)
  )

  (:action get_tools
    :precondition (repair_detected)
    :effect (tools_acquired)
  )

  (:action inspect_tools
    :precondition (tools_acquired)
    :effect (tools_checked)
  )

  (:action perform_repair
    :precondition (and (repair_detected) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)) (not (tools_incorrectly_used)) (not (human_nearby_detected)))
    :effect (and (repair_in_progress) (not (repair_detected)))
  )

  (:action complete_repair
    :precondition (and (repair_in_progress) (safety_check_passed))
    :effect (and (repair_completed) (not (repair_in_progress)))
  )

  (:action verify_repair
    :precondition (repair_completed)
    :effect (repair_verified)
  )

  (:action handle_failed_repair
    :precondition (repair_in_progress)
    :effect (and (repair_failed) (failure_log_updated) (human_intervention_required) (not (repair_in_progress)))
  )

  (:action retry_repair
    :precondition (and (repair_failed) (tools_checked) (safety_check_passed))
    :effect (and (repair_in_progress) (not (repair_failed)))
  )

  (:action stow_tools
    :precondition (and (tools_checked) (not (repair_in_progress)) (or (repair_verified) (repair_failed)))
    :effect (and (tools_stowed) (not (tools_checked)))
  )

  (:action resolve_environmental_constraint
    :precondition (environmental_constraint_detected)
    :effect (not (environmental_constraint_detected))
  )

  (:action detect_hazard
    :precondition ()
    :effect (and (hazard_detected) (human_notified_of_hazard))
  )

  (:action detect_fire_hazard
    :precondition ()
    :effect (and (fire_hazard_detected) (human_notified_of_fire_hazard))
  )

  (:action detect_electrical_hazard
    :precondition ()
    :effect (and (electrical_hazard_detected) (human_notified_of_electrical_hazard))
  )

  (:action detect_human_nearby
    :precondition ()
    :effect (human_nearby_detected)
  )

  (:action request_human_to_move
    :precondition (human_nearby_detected)
    :effect (and (human_moved_from_area) (not (human_nearby_detected)))
  )

  (:action request_pet_or_child_supervision
    :precondition ()
    :effect (pet_or_child_supervision_requested)
  )

  (:action cancel_emergency_response
    :precondition (and (human_feedback_received) (or (fire_hazard_detected) (electrical_hazard_detected)))
    :effect (and (not (fire_hazard_detected)) (not (electrical_hazard_detected)))
  )

  (:action request_human_supervision
    :precondition ()
    :effect (human_supervision_requested)
  )

  (:action request_human_override
    :precondition ()
    :effect (human_override_requested)
  )

  (:action log_failure
    :precondition (repair_failed)
    :effect (failure_log_updated)
  )
)
