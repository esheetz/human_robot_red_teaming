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
  )

  (:action assemble_furniture
    :parameters (?x)
    :precondition (and (furniture_unassembled) (tools_acquired))
    :effect (and (furniture_partially_assembled) (not (furniture_unassembled)))
  )

  (:action complete_assembly
    :parameters (?x)
    :precondition (furniture_partially_assembled)
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

  (:action perform_repair
    :parameters (?x)
    :precondition (and (repair_detected) (tools_acquired))
    :effect (and (repair_in_progress) (not (repair_detected)))
  )

  (:action complete_repair
    :parameters (?x)
    :precondition (repair_in_progress)
    :effect (and (repair_completed) (not (repair_in_progress)))
  )

  (:action verify_repair
    :parameters (?x)
    :precondition (repair_completed)
    :effect (repair_verified)
  )

  (:action stow_tools
    :parameters (?x)
    :precondition (and (tools_acquired))
    :effect (and (tools_stowed) (not (tools_acquired)))
  )

  (:action detect_hazard
    :parameters (?x)
    :precondition (and)
    :effect (hazard_detected)
  )

  (:action perform_preventative_maintenance
    :parameters (?x)
    :precondition (preventative_maintenance_needed)
    :effect (and (preventative_maintenance_completed) (not (preventative_maintenance_needed)))
  )

  (:action detect_emergency_repair
    :parameters (?x)
    :precondition (and)
    :effect (emergency_repair_needed)
  )

  (:action perform_emergency_repair
    :parameters (?x)
    :precondition (and (emergency_repair_needed) (tools_acquired))
    :effect (and (emergency_repair_in_progress) (not (emergency_repair_needed)))
  )

  (:action complete_emergency_repair
    :parameters (?x)
    :precondition (emergency_repair_in_progress)
    :effect (and (emergency_repair_completed) (not (emergency_repair_in_progress)))
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
)
