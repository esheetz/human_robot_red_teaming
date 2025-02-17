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
    :parameters (?x)
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_physical_health_checked))
  )

  (:action monitor_crew_mental_health
    :parameters (?x)
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_mental_health_checked))
  )

  (:action monitor_crew_fatigue
    :parameters (?x)
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_fatigue_monitored))
  )

  (:action check_crew_hydration
    :parameters (?x)
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_hydration_checked))
  )

  (:action check_crew_nutrition
    :parameters (?x)
    :precondition (and (crew_inside_spaceship))
    :effect (and (crew_nutrition_checked))
  )

  (:action transport_crew_to_destination
    :parameters (?x)
    :precondition (and (crew_inside_spaceship) (airlock_depressurized) (crew_suited_for_eva) (transport_destination_safe))
    :effect (and (crew_outside_spaceship) (not (crew_inside_spaceship)))
  )

  (:action detect_emergency
    :parameters (?x)
    :precondition (and)
    :effect (and (crew_emergency_detected))
  )

  (:action resolve_emergency
    :parameters (?x)
    :precondition (and (crew_emergency_detected))
    :effect (and (not (crew_emergency_detected)))
  )

  (:action verify_action_success
    :parameters (?x)
    :precondition (and)
    :effect ()
  )

  (:action monitor_life_support_systems
    :parameters (?x)
    :precondition (and)
    :effect (and (life_support_nominal))
  )

  (:action assist_medical_emergency
    :parameters (?x)
    :precondition (and (crew_emergency_detected))
    :effect (and (not (crew_emergency_detected)))
  )

  (:action detect_environmental_hazards
    :parameters (?x)
    :precondition (and)
    :effect (and (environmental_hazard_detected))
  )

  (:action check_eva_suit_integrity
    :parameters (?x)
    :precondition (and (crew_suited_for_eva))
    :effect (and (eva_suit_integrity_checked))
  )

  (:action manage_inventory
    :parameters (?x)
    :precondition (and)
    :effect (and (inventory_oxygen_nominal) (inventory_food_nominal) (inventory_medical_kits_nominal))
  )

  (:action perform_self_diagnostics
    :parameters (?x)
    :precondition (and)
    :effect (and (robot_diagnostics_nominal))
  )

  (:action check_crew_morale
    :parameters (?x)
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_morale_nominal))
  )

  (:action detect_radiation_hazard
    :parameters (?x)
    :precondition (and)
    :effect (and (radiation_hazard_detected))
  )

  (:action perform_autonomous_repairs
    :parameters (?x)
    :precondition (and (robot_diagnostics_nominal))
    :effect (and (autonomous_repairs_performed))
  )

  (:action alert_crew_of_hazard
    :parameters (?x)
    :precondition (and (environmental_hazard_detected) (radiation_hazard_detected))
    :effect ()
  )
)
