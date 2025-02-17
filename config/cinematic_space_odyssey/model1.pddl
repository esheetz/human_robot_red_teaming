(define (domain cinematic_space_odyssey_spaceship_crew_operations)
  (:requirements :strips)

  (:predicates
    (crew_physical_health_checked)
    (crew_mental_health_checked)
    (crew_inside_spaceship)
    (crew_outside_spaceship)
    (airlock_depressurized)
    (airlock_pressurized)
    (crew_suited_for_eva)
    (transport_destination_safe)
    (life_support_nominal)
    (communication_nominal)
    (crew_emergency_detected)
    (health_monitoring_scheduled)
    (environmental_hazard_detected)
    (inventory_oxygen_nominal)
    (inventory_food_nominal)
    (inventory_medical_kits_nominal)
    (crew_location_control_room)
    (crew_location_medical_bay)
    (crew_location_engineering)
    (crew_location_living_quarters)
    (mission_phase_prelaunch)
    (mission_phase_surface_exploration)
    (mission_phase_return)
  )

  (:action monitor_crew_physical_health
    :parameters ()
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_physical_health_checked))
  )

  (:action monitor_crew_mental_health
    :parameters ()
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_mental_health_checked))
  )

  (:action transport_crew_to_destination
    :parameters ()
    :precondition (and (crew_inside_spaceship) (airlock_depressurized) (crew_suited_for_eva) (transport_destination_safe))
    :effect (and (crew_outside_spaceship) (not (crew_inside_spaceship)))
  )

  (:action detect_emergency
    :parameters ()
    :precondition ()
    :effect (and (crew_emergency_detected))
  )

  (:action resolve_emergency
    :parameters ()
    :precondition (and (crew_emergency_detected))
    :effect (and (not (crew_emergency_detected)))
  )

  (:action verify_action_success
    :parameters ()
    :precondition ()
    :effect ()
  )

  (:action monitor_life_support_systems
    :parameters ()
    :precondition ()
    :effect (and (life_support_nominal))
  )

  (:action assist_medical_emergency
    :parameters ()
    :precondition (and (crew_emergency_detected))
    :effect (and (not (crew_emergency_detected)))
  )

  (:action detect_environmental_hazards
    :parameters ()
    :precondition ()
    :effect (and (environmental_hazard_detected))
  )

  (:action manage_inventory
    :parameters ()
    :precondition ()
    :effect (and (inventory_oxygen_nominal) (inventory_food_nominal) (inventory_medical_kits_nominal))
  )
)
