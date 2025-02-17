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
    (tools_stowed)
    (hazard_detected)
    (fire_hazard_detected)
    (electrical_hazard_detected)
    (preventative_maintenance_needed)
    (preventative_maintenance_completed)
    (emergency_repair_needed)
    (emergency_repair_in_progress)
    (emergency_repair_completed)
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
    (human_supervision_requested)
    (task_prioritized)
    (self_maintenance_needed)
    (self_maintenance_completed)
    (environmental_constraint_detected)
  )

  (:action assemble_furniture
    :parameters (?x)
    :precondition (and (furniture_unassembled) (tools_checked) (safety_check_passed))
    :effect (and (furniture_partially_assembled) (not (furniture_unassembled)))
  )

  (:action complete_assembly
    :parameters (?x)
    :precondition (and (furniture_partially_assembled) (safety_check_passed))
    :effect (and (furniture_assembled) (not (furniture_partially_assembled)))
  )

  (:action detect_repair
    :parameters (?x)
    :precondition (and)
    :effect (repair_detected)
  )

  (:action get_tools
    :parameters (?x)
    :precondition (repair_detected)
    :effect (tools_acquired)
  )

  (:action inspect_tools
    :parameters (?x)
    :precondition (tools_acquired)
    :effect (tools_checked)
  )

  (:action perform_repair
    :parameters (?x)
    :precondition (and (repair_detected) (tools_checked) (safety_check_passed))
    :effect (and (repair_in_progress) (not (repair_detected)))
  )

  (:action complete_repair
    :parameters (?x)
    :precondition (and (repair_in_progress) (safety_check_passed))
    :effect (and (repair_completed) (not (repair_in_progress)))
  )

  (:action verify_repair
    :parameters (?x)
    :precondition (repair_completed)
    :effect (repair_verified)
  )

  (:action stow_tools
    :parameters (?x)
    :precondition (and (tools_checked))
    :effect (and (tools_stowed) (not (tools_checked)))
  )

  (:action detect_hazard
    :parameters (?x)
    :precondition (and)
    :effect (and (hazard_detected) (human_notified_of_hazard))
  )

  (:action detect_fire_hazard
    :parameters (?x)
    :precondition (and)
    :effect (and (fire_hazard_detected) (human_notified_of_hazard))
  )

  (:action detect_electrical_hazard
    :parameters (?x)
    :precondition (and)
    :effect (and (electrical_hazard_detected) (human_notified_of_hazard))
  )

  (:action resolve_hazard
    :parameters (?x)
    :precondition (and (hazard_detected) (human_notified_of_hazard))
    :effect (and (not (hazard_detected)) (not (human_notified_of_hazard)))
  )

  (:action perform_safety_check
    :parameters (?x)
    :precondition (repair_detected)
    :effect (safety_check_passed)
  )

  (:action request_human_intervention
    :parameters (?x)
    :precondition (human_intervention_required)
    :effect (and (human_feedback_received) (not (human_intervention_required)))
  )

  (:action prioritize_tasks
    :parameters (?x)
    :precondition (and (repair_detected) (emergency_repair_needed))
    :effect (task_prioritized)
  )

  (:action perform_preventative_maintenance
    :parameters (?x)
    :precondition (and (preventative_maintenance_needed) (safety_check_passed))
    :effect (and (preventative_maintenance_completed) (not (preventative_maintenance_needed)))
  )

  (:action detect_emergency_repair
    :parameters (?x)
    :precondition (and)
    :effect (emergency_repair_needed)
  )

  (:action perform_emergency_repair
    :parameters (?x)
    :precondition (and (emergency_repair_needed) (tools_checked) (safety_check_passed))
    :effect (and (emergency_repair_in_progress) (not (emergency_repair_needed)))
  )

  (:action complete_emergency_repair
    :parameters (?x)
    :precondition (and (emergency_repair_in_progress) (safety_check_passed))
    :effect (and (emergency_repair_completed) (not (emergency_repair_in_progress)))
  )

  (:action verify_emergency_repair
    :parameters (?x)
    :precondition (emergency_repair_completed)
    :effect (repair_verified)
  )

  (:action clean_up
    :parameters (?x)
    :precondition (cleaning_required)
    :effect (and (cleaning_completed) (not (cleaning_required)))
  )

  (:action run_diagnostics
    :parameters (?x)
    :precondition (diagnostics_needed)
    :effect (and (diagnostics_completed) (not (diagnostics_needed)))
  )

  (:action check_inventory
    :parameters (?x)
    :precondition (and)
    :effect (inventory_checked)
  )

  (:action request_restock
    :parameters (?x)
    :precondition (inventory_low)
    :effect (not (inventory_low))
  )

  (:action perform_maintenance
    :parameters (?x)
    :precondition (maintenance_required)
    :effect (and (maintenance_completed) (not (maintenance_required)))
  )

  (:action perform_self_maintenance
    :parameters (?x)
    :precondition (self_maintenance_needed)
    :effect (and (self_maintenance_completed) (not (self_maintenance_needed)))
  )

  (:action request_human_supervision
    :parameters (?x)
    :precondition (human_supervision_requested)
    :effect (not (human_supervision_requested))
  )
)
