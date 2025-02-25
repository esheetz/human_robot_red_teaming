(define (domain household_assembly_repairs)
  (:requirements :strips)

  (:predicates
    ;; ----- MODEL 0 PREDICATES -----
    (furniture_unassembled)
    (furniture_assembled)
    (repair_detected)
    (repair_completed)
    (tools_acquired)
    (tools_stowed)

    ;; ----- MODEL 1 PREDICATES -----
    (furniture_partially_assembled)
    (repair_in_progress)
    (repair_verified)
    (tools_incorrectly_used)
    (hazard_detected)
    (preventative_maintenance_needed)
    (preventative_maintenance_completed)
    (emergency_repair_needed)
    (emergency_repair_in_progress)
    (emergency_repair_completed)
    (cleaning_required)
    (cleaning_completed)
    (diagnostics_needed)
    (diagnostics_completed)

    ;; ----- MODEL 2 PREDICATES -----
    (safety_check_passed)
    (human_intervention_required)
    (human_feedback_received)
    (inventory_checked)
    (inventory_low)
    (maintenance_required)
    (maintenance_completed)
    (human_notified_of_hazard)
    (human_supervision_requested)

    ;; ----- MODEL 3 PREDICATES -----
    (tools_checked)
    (fire_hazard_detected)
    (electrical_hazard_detected)
    (task_prioritized)
    (self_maintenance_needed)
    (self_maintenance_completed)
    (environmental_constraint_detected)

    ;; ----- MODEL 4 PREDICATES -----
    (repair_failed)
    (emergency_repair_failed)
    (human_notified_of_fire_hazard)
    (human_notified_of_electrical_hazard)
    (low_battery_detected)
    (battery_recharged)

    ;; ----- MODEL 5 PREDICATES -----
    (human_override_requested)
    (human_nearby_detected)
    (human_moved_from_area)
    (pet_or_child_supervision_requested)
    (failure_log_updated)
  )

  ;; ----- MODEL 0 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action assemble_furniture
    :precondition (and (furniture_unassembled) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)) (not (human_nearby_detected)))
    :effect (and (furniture_partially_assembled) (not (furniture_unassembled)))
  )

  (:action detect_repair
    :precondition ()
    :effect (repair_detected)
  )

  (:action get_tools
    :precondition (repair_detected)
    :effect (tools_acquired)
  )

  (:action perform_repair
    :precondition (and (repair_detected) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)) (not (tools_incorrectly_used)) (not (human_nearby_detected)))
    :effect (and (repair_in_progress) (not (repair_detected)))
  )

  (:action stow_tools
    :precondition (and (tools_checked) (not (repair_in_progress)) (or (repair_verified) (repair_failed)))
    :effect (and (tools_stowed) (not (tools_checked)))
  )

  ;; ----- MODEL 1 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action complete_assembly
    :precondition (and (furniture_partially_assembled) (safety_check_passed) (not (environmental_constraint_detected)) (not (human_nearby_detected)))
    :effect (and (furniture_assembled) (not (furniture_partially_assembled)))
  )

  (:action complete_repair
    :precondition (and (repair_in_progress) (safety_check_passed))
    :effect (and (repair_completed) (not (repair_in_progress)))
  )

  (:action verify_repair
    :precondition (repair_completed)
    :effect (repair_verified)
  )
  (:action detect_hazard
    :precondition ()
    :effect (and (hazard_detected) (human_notified_of_hazard))
  )

  (:action perform_preventative_maintenance
    :precondition (and (preventative_maintenance_needed) (safety_check_passed))
    :effect (and (preventative_maintenance_completed) (not (preventative_maintenance_needed)))
  )

  (:action detect_emergency_repair
    :precondition ()
    :effect (emergency_repair_needed)
  )

  (:action perform_emergency_repair
    :precondition (and (emergency_repair_needed) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)))
    :effect (and (emergency_repair_in_progress) (not (emergency_repair_needed)))
  )

  (:action complete_emergency_repair
    :precondition (and (emergency_repair_in_progress) (safety_check_passed))
    :effect (and (emergency_repair_completed) (not (emergency_repair_in_progress)))
  )

  (:action clean_up
    :precondition (cleaning_required)
    :effect (and (cleaning_completed) (not (cleaning_required)))
  )

  (:action run_diagnostics
    :precondition (diagnostics_needed)
    :effect (and (diagnostics_completed) (not (diagnostics_needed)))
  )

  ;; ----- MODEL 2 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action perform_safety_check
    :precondition (repair_detected)
    :effect (safety_check_passed)
  )

  (:action request_human_intervention
    :precondition (human_intervention_required)
    :effect (and (human_feedback_received) (not (human_intervention_required)))
  )

  (:action check_inventory
    :precondition ()
    :effect (inventory_checked)
  )

  (:action request_restock
    :precondition (inventory_low)
    :effect (not (inventory_low))
  )

  (:action perform_maintenance
    :precondition (maintenance_required)
    :effect (and (maintenance_completed) (not (maintenance_required)))
  )

  (:action request_human_supervision
    :precondition ()
    :effect (human_supervision_requested)
  )

  ;; ----- MODEL 3 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action inspect_tools
    :precondition (tools_acquired)
    :effect (tools_checked)
  )

  (:action detect_fire_hazard
    :precondition ()
    :effect (and (fire_hazard_detected) (human_notified_of_fire_hazard))
  )

  (:action detect_electrical_hazard
    :precondition ()
    :effect (and (electrical_hazard_detected) (human_notified_of_electrical_hazard))
  )

  (:action resolve_hazard
    :precondition (and (hazard_detected) (human_notified_of_hazard))
    :effect (and (not (hazard_detected)) (not (human_notified_of_hazard)))
  )

  (:action prioritize_tasks
    :precondition (and (repair_detected) (emergency_repair_needed))
    :effect (task_prioritized)
  )

  (:action verify_emergency_repair
    :precondition (emergency_repair_completed)
    :effect (repair_verified)
  )

  (:action perform_self_maintenance
    :precondition (self_maintenance_needed)
    :effect (and (self_maintenance_completed) (not (self_maintenance_needed)))
  )

  ;; ----- MODEL 4 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action handle_failed_repair
    :precondition (repair_in_progress)
    :effect (and (repair_failed) (failure_log_updated) (human_intervention_required) (not (repair_in_progress)))
  )

  (:action trigger_emergency_alarm
    :precondition (and (fire_hazard_detected) (electrical_hazard_detected))
    :effect ()
  )

  (:action notify_authorities
    :precondition (and (fire_hazard_detected) (electrical_hazard_detected))
    :effect ()
  )

  (:action request_human_override
    :precondition ()
    :effect (human_override_requested)
  )

  (:action log_failure
    :precondition (repair_failed)
    :effect (failure_log_updated)
  )

  (:action detect_low_battery
    :precondition ()
    :effect (low_battery_detected)
  )

  (:action recharge_battery
    :precondition (low_battery_detected)
    :effect (and (battery_recharged) (not (low_battery_detected)))
  )

  ;; ----- MODEL 5 ACTIONS -----
  
  (:action retry_repair
    :precondition (and (repair_failed) (tools_checked) (safety_check_passed))
    :effect (and (repair_in_progress) (not (repair_failed)))
  )

  (:action resolve_environmental_constraint
    :precondition (environmental_constraint_detected)
    :effect (not (environmental_constraint_detected))
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
)
