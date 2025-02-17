(define (domain cinematic_space_odyssey_spaceship_crew_operations)
  (:requirements :strips)

  (:predicates
    (crew_physical_health_checked)
    (crew_mental_health_checked)
    (crew_inside_spaceship)
    (crew_outside_spaceship)
  )

  (:action monitor_crew_physical_health
    :parameters (?x)
    :precondition (and)
    :effect (and (crew_physical_health_checked))
  )

  (:action monitor_crew_mental_health
    :parameters (?x)
    :precondition (and)
    :effect (and (crew_mental_health_checked))
  )

  (:action transport_crew_to_destination
    :parameters (?x)
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_outside_spaceship) (not (crew_inside_spaceship)))
  )
)
