(define (domain cinematic_space_odyssey_spaceship_crew_operations)
  (:requirements :strips)

  (:predicates
    ;; ----- MODEL 0 PREDICATES -----
    (crew_physical_health_checked)
    (crew_mental_health_checked)
    (crew_inside_spaceship)
    (crew_outside_spaceship)

    ;; ----- MODEL 1 PREDICATES -----
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

    ;; ----- MODEL 2 PREDICATES -----
    (crew_fatigue_monitored)
    (crew_hydration_checked)
    (crew_nutrition_checked)
    (radiation_hazard_detected)
    (robot_diagnostics_nominal)
    (crew_morale_nominal)
    (eva_suit_integrity_checked)
    (autonomous_repairs_performed)

    ;; ----- MODEL 3 PREDICATES -----
    (life_support_failure_detected)
    (systems_malfunction_detected)
    (robot_malfunction_detected)
    (medical_emergency_resolved)
    (hazard_response_initiated)
    (hazard_communicated_to_ground_control)
    (crew_override_active)
    (ground_control_override_active)
    (human_verification_received)

    ;; ----- MODEL 4 PREDICATES -----
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

    ;; ----- MODEL 5 PREDICATES -----
    (communication_delayed)
    (hazard_mitigation_verified)
    (emergency_resolved_verified)
    (environmental_conditions_tracked)
    (environmental_conditions_reported)
  )

  ;; ----- MODEL 0 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

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

  ;; ----- MODEL 1 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action detect_emergency
    :precondition ()
    :effect (and (crew_emergency_detected))
  )

  (:action resolve_emergency
    :precondition (and (crew_emergency_detected) (medical_intervention_verified))
    :effect (and (medical_emergency_resolved) (emergency_resolved_verified) (not (crew_emergency_detected)))
  )

  (:action verify_action_success
    :precondition (and (human_verification_received))
    :effect (and (crew_transport_verified) (medical_intervention_verified) (hazard_mitigation_successful) (hazard_mitigation_verified))
  )

  (:action monitor_life_support_systems
    :precondition ()
    :effect (and (life_support_nominal) (not (life_support_failure_detected)))
  )

  (:action assist_medical_emergency
    :precondition (and (crew_emergency_detected))
    :effect (and (medical_intervention_verified))
  )

  (:action detect_environmental_hazards
    :precondition ()
    :effect (and (environmental_hazard_detected))
  )

  (:action manage_inventory
    :precondition ()
    :effect (and (inventory_oxygen_nominal) (inventory_food_nominal) (inventory_medical_kits_nominal))
  )

  ;; ----- MODEL 2 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

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

  (:action check_eva_suit_integrity
    :precondition (and (crew_suited_for_eva))
    :effect (and (eva_suit_integrity_checked))
  )

  (:action perform_self_diagnostics
    :precondition ()
    :effect (and (robot_diagnostics_nominal) (not (robot_malfunction_detected)))
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
    :precondition (and (robot_diagnostics_nominal) (systems_malfunction_detected))
    :effect (and (autonomous_repairs_performed) (not (systems_malfunction_detected)))
  )

  (:action alert_crew_of_hazard
    :precondition (and (environmental_hazard_detected) (radiation_hazard_detected))
    :effect (and (hazard_response_initiated))
  )

  ;; ----- MODEL 3 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action communicate_hazard_to_ground_control
    :precondition (and (hazard_response_initiated))
    :effect (and (hazard_communicated_to_ground_control))
  )

  (:action crew_override
    :precondition ()
    :effect (and (crew_override_active))
  )

  (:action ground_control_override
    :precondition ()
    :effect (and (ground_control_override_active))
  )

  ;; ----- MODEL 4 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

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

  ;; ----- MODEL 5 ACTIONS -----

  (:action mitigate_hazard
    :precondition (and (environmental_hazard_detected))
    :effect (and (hazard_mitigation_successful) (not (environmental_hazard_detected)))
  )

  (:action resolve_radiation_hazard
    :precondition (and (radiation_hazard_detected))
    :effect (and (hazard_mitigation_successful) (not (radiation_hazard_detected)))
  )

  (:action restore_life_support
    :precondition (and (life_support_failure_detected))
    :effect (and (life_support_nominal) (not (life_support_failure_detected)))
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
