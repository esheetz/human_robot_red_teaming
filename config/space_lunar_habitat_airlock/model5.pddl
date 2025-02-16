(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    ;; Robot and astronaut locations
    (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat)
    (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)

    ;; Key states
    (key_in_habitat) (key_in_airlock) (key_with_robot) (key_with_astronaut)
    (backup_key_available)

    ;; Door states
    (door_habitat_airlock_locked_closed) (door_habitat_airlock_unlocked_opened)
    (door_airlock_surface_locked_closed) (door_airlock_surface_unlocked_opened)

    ;; Door operational states
    (door_habitat_airlock_operational) (door_habitat_airlock_faulty)
    (door_airlock_surface_operational) (door_airlock_surface_faulty)

    ;; Airlock pressure states
    (airlock_pressurized) (airlock_depressurized)

    ;; Robot power states
    (robot_power_normal) (robot_power_low) (robot_power_charging)

    ;; Robot system health
    (robot_system_nominal) (robot_system_fault)

    ;; Emergency states
    (airlock_breach_detected) (no_airlock_breach)
    (habitat_depressurization_alarm) (no_habitat_depressurization_alarm)
    (emergency_communication_active) (emergency_resolved) (emergency_acknowledged_by_astronaut)

    ;; Habitat system failures
    (air_filter_fault) (solar_panel_fault) (temperature_control_fault)
    (habitat_systems_nominal)

    ;; Environmental hazard states
    (environmental_hazard_detected) (no_environmental_hazard)
    (lunar_dust_contamination_detected) (no_lunar_dust_contamination)
    (temperature_variation_detected) (no_temperature_variation)

    ;; Astronaut health monitoring
    (astronaut_health_alert) (no_astronaut_health_alert)
    (astronaut_medical_response_initiated)

    ;; Structural integrity inspection
    (structural_integrity_check_required) (structural_integrity_check_completed)

    ;; Lunar sample handling
    (lunar_sample_on_surface) (lunar_sample_with_robot) (lunar_sample_in_habitat)
    (astronaut_approved_sample_placement)

    ;; Habitat maintenance
    (habitat_maintenance_required) (habitat_maintenance_completed)
  )

  ;; Finding a key
  (:action find_key
    :precondition (and (or (robot_inside_habitat) (robot_inside_airlock))
                       (or (key_with_astronaut) (key_in_habitat) (key_in_airlock) (backup_key_available)))
    :effect (and (key_with_robot)
                 (not (key_with_astronaut)) (not (key_in_habitat)) (not (key_in_airlock)) (not (backup_key_available)))
  )

  ;; Astronaut gives key to robot
  (:action astronaut_give_key_to_robot
    :precondition (and (key_with_astronaut)
                       (or (astronaut_inside_habitat) (astronaut_inside_airlock))
                       (or (robot_inside_habitat) (robot_inside_airlock)))
    :effect (and (key_with_robot) (not (key_with_astronaut)))
  )

  ;; Resolving emergency communication
  (:action resolve_emergency_communication
    :precondition (and (emergency_communication_active) (emergency_acknowledged_by_astronaut) (robot_system_nominal))
    :effect (and (emergency_resolved) (not (emergency_communication_active)) (not (emergency_acknowledged_by_astronaut)))
  )

  ;; Acknowledge emergency by astronaut
  (:action acknowledge_emergency
    :precondition (and (emergency_communication_active)
                       (or (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)))
    :effect (emergency_acknowledged_by_astronaut)
  )

  ;; Monitor astronaut health
  (:action monitor_astronaut_health
    :precondition (and (or (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface))
                       (robot_system_nominal))
    :effect (or (astronaut_health_alert) (no_astronaut_health_alert))
  )

  ;; Initiate medical response for astronaut
  (:action initiate_medical_response
    :precondition (and (astronaut_health_alert)
                       (or (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)))
    :effect (and (astronaut_medical_response_initiated) (not (astronaut_health_alert)))
  )

  ;; Perform habitat maintenance
  (:action perform_habitat_maintenance
    :precondition (and (robot_inside_habitat) (habitat_maintenance_required) (robot_system_nominal))
    :effect (and (habitat_maintenance_completed) (habitat_systems_nominal)
                 (not (habitat_maintenance_required))
                 (not (air_filter_fault)) (not (solar_panel_fault)) (not (temperature_control_fault)))
  )

  ;; Inspect structural integrity
  (:action inspect_structural_integrity
    :precondition (and (structural_integrity_check_required)
                       (or (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat))
                       (robot_system_nominal))
    :effect (and (structural_integrity_check_completed) (not (structural_integrity_check_required)))
  )

  ;; Inspect environmental factors
  (:action inspect_environmental_factors
    :precondition (and (robot_outside_habitat) (robot_system_nominal))
    :effect (or (lunar_dust_contamination_detected) (no_lunar_dust_contamination)
                (temperature_variation_detected) (no_temperature_variation))
  )

  ;; Assist emergency communication
  (:action assist_emergency_communication
    :precondition (and (emergency_communication_active) (robot_system_nominal))
    :effect (emergency_communication_active) ;; Maintain state until resolved
  )

  ;; Recharge battery
  (:action recharge_battery
    :precondition (and (robot_power_low) (or (robot_inside_habitat) (robot_inside_airlock)))
    :effect (and (robot_power_charging) (not (robot_power_low)))
  )

  ;; Complete battery recharge
  (:action complete_battery_recharge
    :precondition (robot_power_charging)
    :effect (and (robot_power_normal) (not (robot_power_charging)))
  )

  ;; Enter safe mode due to low power
  (:action enter_safe_mode_due_to_low_power
    :precondition (robot_power_low)
    :effect (and (robot_system_fault) (not (robot_system_nominal)))
  )
)
