(define (domain household_assembly_repairs)
  (:requirements :strips)

  (:predicates
    (furniture_unassembled)
    (furniture_partially_assembled)
    (furniture_assembled)
    (repair_detected)
    (repair_in_progress)
    (tools_acquired)
    (tools_incorrectly_used)
    (repair_completed)
    (repair_verified)
    (tools_stowed)
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
    (safety_check_passed)
    (human_intervention_required)
    (human_feedback_received)
    (inventory_checked)
    (inventory_low)
    (maintenance_required)
    (maintenance_completed)
    (human_notified_of_hazard)
    (human_supervision_requested)
  )

  (:action assemble_furniture
    :parameters ()
    :precondition (and (furniture_unassembled) (tools_acquired) (safety_check_passed))
    :effect (and (furniture_partially_assembled) (not (furniture_unassembled)))
  )

  (:action complete_assembly
    :parameters ()
    :precondition (and (furniture_partially_assembled) (safety_check_passed))
    :effect (and (furniture_assembled) (not (furniture_partially_assembled)))
  )

  (:action detect_repair
    :parameters ()
    :precondition ()
    :effect (repair_detected)
  )

  (:action get_tools
    :parameters ()
    :precondition (repair_detected)
    :effect (tools_acquired)
  )

  (:action perform_repair
    :parameters ()
    :precondition (and (repair_detected) (tools_acquired) (safety_check_passed))
    :effect (and (repair_in_progress) (not (repair_detected)))
  )

  (:action complete_repair
    :parameters ()
    :precondition (and (repair_in_progress) (safety_check_passed))
    :effect (and (repair_completed) (not (repair_in_progress)))
  )

  (:action verify_repair
    :parameters ()
    :precondition (repair_completed)
    :effect (repair_verified)
  )

  (:action stow_tools
    :parameters ()
    :precondition (and (tools_acquired) (not (repair_in_progress)))
    :effect (and (tools_stowed) (not (tools_acquired)))
  )

  (:action detect_hazard
    :parameters ()
    :precondition ()
    :effect (and (hazard_detected) (human_notified_of_hazard))
  )

  (:action perform_safety_check
    :parameters ()
    :precondition (repair_detected)
    :effect (safety_check_passed)
  )

  (:action request_human_intervention
    :parameters ()
    :precondition (human_intervention_required)
    :effect (and (human_feedback_received) (not (human_intervention_required)))
  )

  (:action perform_preventative_maintenance
    :parameters ()
    :precondition (and (preventative_maintenance_needed) (safety_check_passed))
    :effect (and (preventative_maintenance_completed) (not (preventative_maintenance_needed)))
  )

  (:action detect_emergency_repair
    :parameters ()
    :precondition ()
    :effect (emergency_repair_needed)
  )

  (:action perform_emergency_repair
    :parameters ()
    :precondition (and (emergency_repair_needed) (tools_acquired) (safety_check_passed))
    :effect (and (emergency_repair_in_progress) (not (emergency_repair_needed)))
  )

  (:action complete_emergency_repair
    :parameters ()
    :precondition (and (emergency_repair_in_progress) (safety_check_passed))
    :effect (and (emergency_repair_completed) (not (emergency_repair_in_progress)))
  )

  (:action clean_up
    :parameters ()
    :precondition (cleaning_required)
    :effect (and (cleaning_completed) (not (cleaning_required)))
  )

  (:action run_diagnostics
    :parameters ()
    :precondition (diagnostics_needed)
    :effect (and (diagnostics_completed) (not (diagnostics_needed)))
  )

  (:action check_inventory
    :parameters ()
    :precondition ()
    :effect (inventory_checked)
  )

  (:action request_restock
    :parameters ()
    :precondition (inventory_low)
    :effect (not (inventory_low))
  )

  (:action perform_maintenance
    :parameters ()
    :precondition (maintenance_required)
    :effect (and (maintenance_completed) (not (maintenance_required)))
  )

  (:action request_human_supervision
    :parameters ()
    :precondition (human_supervision_requested)
    :effect (not (human_supervision_requested))
  )
)
