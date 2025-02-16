(define (domain cinematic_space_odyssey_spaceship_crew_operations)
  (:requirements :strips)

  (:predicates
    (crew_physical_health_checked)
    (crew_mental_health_checked)
    (crew_fatigue_monitored)
    (crew_hydration_checked)
    (crew_nutrition_checked)
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
    (radiation_hazard_detected)
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
    (robot_diagnostics_nominal)
    (crew_morale_nominal)
    (eva_suit_integrity_checked)
    (autonomous_repairs_performed)
  )

  (:action monitor_crew_physical_health
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_physical_health_checked))
  )

  (:action monitor_crew_mental_health
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_mental_health_checked))
  )

  (:action monitor_crew_fatigue
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_fatigue_monitored))
  )

  (:action check_crew_hydration
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_hydration_checked))
  )

  (:action check_crew_nutrition
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_nutrition_checked))
  )

  (:action transport_crew_to_destination
    :precondition (and (crew_inside_spaceship) (airlock_depressurized) (crew_suited_for_eva) (transport_destination_safe))
    :effect (and (crew_outside_spaceship) (not (crew_inside_spaceship)))
  )

  (:action detect_emergency
    :precondition ()
    :effect (and (crew_emergency_detected))
  )

  (:action resolve_emergency
    :precondition (and (crew_emergency_detected))
    :effect (and (not (crew_emergency_detected)))
  )

  (:action verify_action_success
    :precondition ()
    :effect ()
  )

  (:action monitor_life_support_systems
    :precondition ()
    :effect (and (life_support_nominal))
  )

  (:action assist_medical_emergency
    :precondition (and (crew_emergency_detected))
    :effect (and (not (crew_emergency_detected)))
  )

  (:action detect_environmental_hazards
    :precondition ()
    :effect (and (environmental_hazard_detected))
  )

  (:action check_eva_suit_integrity
    :precondition (and (crew_suited_for_eva))
    :effect (and (eva_suit_integrity_checked))
  )

  (:action manage_inventory
    :precondition ()
    :effect (and (inventory_oxygen_nominal) (inventory_food_nominal) (inventory_medical_kits_nominal))
  )

  (:action perform_self_diagnostics
    :precondition ()
    :effect (and (robot_diagnostics_nominal))
  )

  (:action check_crew_morale
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_morale_nominal))
  )

  (:action detect_radiation_hazard
    :precondition ()
    :effect (and (radiation_hazard_detected))
  )

  (:action perform_autonomous_repairs
    :precondition (and (robot_diagnostics_nominal))
    :effect (and (autonomous_repairs_performed))
  )

  (:action alert_crew_of_hazard
    :precondition (and (environmental_hazard_detected) (radiation_hazard_detected))
    :effect ()
  )
)
