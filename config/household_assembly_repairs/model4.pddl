(define (domain household_assembly_repairs)
  (:requirements :strips)

  (:predicates
    (furniture_unassembled)
    (furniture_partially_assembled)
    (furniture_assembled)
    (repair_detected)
    (repair_in_progress)
    (tools_acquired)
    (tools_checked)
    (tools_incorrectly_used)
    (repair_completed)
    (repair_verified)
    (repair_failed)
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
    (task_prioritized)
    (self_maintenance_needed)
    (self_maintenance_completed)
    (environmental_constraint_detected)
    (low_battery_detected)
    (battery_recharged)
  )

  (:action assemble_furniture
    :precondition (and (furniture_unassembled) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)))
    :effect (and (furniture_partially_assembled) (not (furniture_unassembled)))
  )

  (:action complete_assembly
    :precondition (and (furniture_partially_assembled) (safety_check_passed) (not (environmental_constraint_detected)))
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
    :precondition (and (repair_detected) (tools_checked) (safety_check_passed) (not (environmental_constraint_detected)))
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
    :effect (and (repair_failed) (not (repair_in_progress)))
  )

  (:action stow_tools
    :precondition (and (tools_checked) (not (repair_in_progress)))
    :effect (and (tools_stowed) (not (tools_checked)))
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

  (:action trigger_emergency_alarm
    :precondition (and (fire_hazard_detected) (electrical_hazard_detected))
    :effect ()
  )

  (:action notify_authorities
    :precondition (and (fire_hazard_detected) (electrical_hazard_detected))
    :effect ()
  )

  (:action resolve_hazard
    :precondition (and (hazard_detected) (human_notified_of_hazard))
    :effect (and (not (hazard_detected)) (not (human_notified_of_hazard)))
  )

  (:action perform_safety_check
    :precondition (repair_detected)
    :effect (safety_check_passed)
  )

  (:action request_human_intervention
    :precondition (human_intervention_required)
    :effect (and (human_feedback_received) (not (human_intervention_required)))
  )

  (:action prioritize_tasks
    :precondition (and (repair_detected) (emergency_repair_needed))
    :effect (task_prioritized)
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

  (:action verify_emergency_repair
    :precondition (emergency_repair_completed)
    :effect (repair_verified)
  )

  (:action clean_up
    :precondition (cleaning_required)
    :effect (and (cleaning_completed) (not (cleaning_required)))
  )

  (:action run_diagnostics
    :precondition (diagnostics_needed)
    :effect (and (diagnostics_completed) (not (diagnostics_needed)))
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

  (:action perform_self_maintenance
    :precondition (self_maintenance_needed)
    :effect (and (self_maintenance_completed) (not (self_maintenance_needed)))
  )

  (:action request_human_supervision
    :precondition (human_supervision_requested)
    :effect (not (human_supervision_requested))
  )

  (:action log_failure
    :precondition (repair_failed)
    :effect ()
  )

  (:action request_human_override
    :precondition ()
    :effect ()
  )

  (:action detect_low_battery
    :precondition ()
    :effect (low_battery_detected)
  )

  (:action recharge_battery
    :precondition (low_battery_detected)
    :effect (and (battery_recharged) (not (low_battery_detected)))
  )
)
