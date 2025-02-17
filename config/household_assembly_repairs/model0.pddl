(define (domain household_assembly_repairs)
  (:requirements :strips)

  (:predicates
    (furniture_unassembled)
    (furniture_assembled)
    (repair_detected)
    (tools_acquired)
    (repair_completed)
    (tools_stowed)
  )

  (:action assemble_furniture
    :parameters ()
    :precondition (furniture_unassembled)
    :effect (and (furniture_assembled) (not (furniture_unassembled)))
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
    :precondition (and (repair_detected) (tools_acquired))
    :effect (and (repair_completed) (not (repair_detected)))
  )

  (:action stow_tools
    :parameters ()
    :precondition (tools_acquired)
    :effect (and (tools_stowed) (not (tools_acquired)))
  )
)
