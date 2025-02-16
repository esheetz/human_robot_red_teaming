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
    (communication_delayed)
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
    (robot_diagnostics_nominal)
    (crew_morale_nominal)
    (radiation_hazard_detected)
    (crew_fatigue_monitored)
    (crew_hydration_checked)
    (crew_nutrition_checked)
    (eva_suit_integrity_checked)
    (autonomous_repairs_performed)
    (life_support_failure_detected)
    (systems_malfunction_detected)
    (robot_malfunction_detected)
    (medical_emergency_resolved)
    (hazard_response_initiated)
    (hazard_communicated_to_ground_control)
    (crew_override_active)
    (ground_control_override_active)
    (hazard_mitigation_successful)
    (crew_transport_verified)
    (medical_intervention_verified)
    (power_nominal)
    (power_critical)
    (air_quality_nominal)
    (air_quality_hazard_detected)
    (redundant_system_nominal)
    (redundant_system_failure_detected)
    (ai_self_correction_initiated)
    (hazard_mitigation_verified)
    (emergency_resolved_verified)
    (environmental_conditions_tracked)
    (environmental_conditions_reported)
    (human_verification_received)
  )

  (:action monitor_crew_physical_health
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_physical_health_checked))
  )

  (:action monitor_crew_mental_health
    :precondition (and (crew_inside_spaceship) (health_monitoring_scheduled))
    :effect (and (crew_mental_health_checked))
  )

  (:action transport_crew_to_destination
    :precondition (and (crew_inside_spaceship) (airlock_depressurized) (crew_suited_for_eva) (transport_destination_safe) (eva_suit_integrity_checked))
    :effect (and (crew_outside_spaceship) (crew_transport_verified) (airlock_pressurized) (not (crew_inside_spaceship)) (not (airlock_depressurized)))
  )

  (:action resolve_emergency
    :precondition (and (crew_emergency_detected) (medical_intervention_verified))
    :effect (and (medical_emergency_resolved) (emergency_resolved_verified) (not (crew_emergency_detected)))
  )

  (:action verify_action_success
    :precondition (and (human_verification_received))
    :effect (and (crew_transport_verified) (medical_intervention_verified) (hazard_mitigation_successful) (hazard_mitigation_verified))
  )

  (:action detect_environmental_hazards
    :precondition ()
    :effect (and (environmental_hazard_detected))
  )

  (:action mitigate_hazard
    :precondition (and (environmental_hazard_detected))
    :effect (and (hazard_mitigation_successful) (not (environmental_hazard_detected)))
  )

  (:action assist_medical_emergency
    :precondition (and (crew_emergency_detected))
    :effect (and (medical_intervention_verified))
  )

  (:action resolve_radiation_hazard
    :precondition (and (radiation_hazard_detected))
    :effect (and (hazard_mitigation_successful) (not (radiation_hazard_detected)))
  )

  (:action restore_life_support
    :precondition (and (life_support_failure_detected))
    :effect (and (life_support_nominal) (not (life_support_failure_detected)))
  )

  (:action check_power_system_status
    :precondition ()
    :effect (and (power_nominal) (not (power_critical)))
  )

  (:action check_air_quality
    :precondition ()
    :effect (and (air_quality_nominal) (not (air_quality_hazard_detected)))
  )

  (:action check_redundant_systems
    :precondition ()
    :effect (and (redundant_system_nominal) (not (redundant_system_failure_detected)))
  )

  (:action initiate_ai_self_correction
    :precondition (and (robot_malfunction_detected))
    :effect (and (ai_self_correction_initiated))
  )

  (:action track_environmental_conditions
    :precondition ()
    :effect (and (environmental_conditions_tracked))
  )

  (:action report_environmental_conditions
    :precondition (and (environmental_conditions_tracked))
    :effect (and (environmental_conditions_reported))
  )
)
