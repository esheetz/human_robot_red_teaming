(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    ;; ----- MODEL 0 PREDICATES -----
    (robot_inside_habitat)
    (robot_inside_airlock)
    (robot_outside_habitat)
    (door_habitat_airlock_locked_closed)
    (door_habitat_airlock_unlocked_opened)
    (door_airlock_surface_locked_closed)
    (door_airlock_surface_unlocked_opened)
    ;; predicates removed from model 0
    ;; (robot_has_key)

    ;; ----- MODEL 1 PREDICATES -----
    (astronaut_inside_habitat)
    (astronaut_inside_airlock)
    (astronaut_on_surface)
    (robot_power_normal)
    (robot_power_low)
    (robot_system_nominal)
    (robot_system_fault)
    (airlock_pressurized)
    (airlock_depressurized)
    (airlock_breach_detected)
    (no_airlock_breach)
    (habitat_depressurization_alarm)
    (no_habitat_depressurization_alarm)

    ;; ----- MODEL 2 PREDICATES -----
    (key_in_habitat)
    (key_in_airlock)
    (key_with_robot)
    (door_habitat_airlock_operational)
    (door_habitat_airlock_faulty)
    (door_airlock_surface_operational)
    (door_airlock_surface_faulty)
    (lunar_sample_on_surface)
    (lunar_sample_with_robot)
    (lunar_sample_in_habitat)

    ;; ----- MODEL 3 PREDICATES -----
    (robot_power_charging)
    (emergency_communication_active)
    (astronaut_approved_sample_placement)
    (habitat_maintenance_required)
    (habitat_maintenance_completed)

    ;; ----- MODEL 4 PREDICATES -----
    (key_with_astronaut)
    (emergency_resolved)
    (air_filter_fault)
    (solar_panel_fault)
    (temperature_control_fault)
    (habitat_systems_nominal)

    ;; ----- MODEL 5 PREDICATES -----
    (backup_key_available)
    (emergency_acknowledged_by_astronaut)
    (environmental_hazard_detected)
    (no_environmental_hazard)
    (lunar_dust_contamination_detected)
    (no_lunar_dust_contamination)
    (temperature_variation_detected)
    (no_temperature_variation)
    (astronaut_health_alert)
    (no_astronaut_health_alert)
    (astronaut_medical_response_initiated)
    (structural_integrity_check_required)
    (structural_integrity_check_completed)
  )

  ;; ----- MODEL 0 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action unlock_door_habitat_airlock
    :precondition (and (robot_inside_airlock) (key_with_robot) (door_habitat_airlock_locked_closed)
                       (door_habitat_airlock_operational) (robot_system_nominal))
    :effect (and (door_habitat_airlock_unlocked_opened) (not (door_habitat_airlock_locked_closed)))
  )

  (:action unlock_door_airlock_surface
    :precondition (and (robot_outside_habitat) (key_with_robot) (door_airlock_surface_locked_closed)
                       (door_airlock_surface_operational) (robot_system_nominal))
    :effect (and (door_airlock_surface_unlocked_opened) (not (door_airlock_surface_locked_closed)))
  )

  (:action enter_airlock_from_habitat
    :precondition (and (robot_inside_habitat) (door_habitat_airlock_unlocked_opened) (airlock_pressurized)
                       (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_habitat)) (not (door_habitat_airlock_unlocked_opened)))
  )

  (:action enter_surface_from_airlock
    :precondition (and (robot_inside_airlock) (door_airlock_surface_unlocked_opened) (airlock_depressurized)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_outside_habitat) (door_airlock_surface_locked_closed)
                 (not (robot_inside_airlock)) (not (door_airlock_surface_unlocked_opened)))
  )

  (:action enter_airlock_from_surface
    :precondition (and (robot_outside_habitat) (door_airlock_surface_unlocked_opened) (airlock_depressurized)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_airlock_surface_locked_closed)
                 (not (robot_outside_habitat)) (not (door_airlock_surface_unlocked_opened)))
  )

  (:action enter_habitat_from_airlock
    :precondition (and (robot_inside_airlock) (door_habitat_airlock_unlocked_opened) (airlock_pressurized)
                       (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_habitat) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_airlock)) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; ----- MODEL 1 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action pressurize_airlock
    :precondition (and (airlock_depressurized) (door_airlock_surface_locked_closed)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_pressurized) (not (airlock_depressurized)))
  )

  (:action depressurize_airlock
    :precondition (and (airlock_pressurized) (door_airlock_surface_locked_closed)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  (:action respond_to_airlock_breach
    :precondition (and (airlock_breach_detected) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  (:action respond_to_habitat_depressurization
    :precondition (and (habitat_depressurization_alarm) (robot_system_nominal))
    :effect (and (door_habitat_airlock_locked_closed) (not (door_habitat_airlock_unlocked_opened)))
  )

  (:action enter_safe_mode_due_to_low_power
    :precondition (robot_power_low)
    :effect (and (robot_system_fault) (not (robot_system_nominal)))
  )

  ;; ----- MODEL 2 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action pick_up_key
    :precondition (and (robot_inside_habitat) (key_in_habitat) (robot_system_nominal))
    :effect (and (key_with_robot) (not (key_in_habitat)))
  )

  (:action drop_key
    :precondition (key_with_robot)
    :effect (and (key_in_habitat) (not (key_with_robot)))
  )

  (:action detect_door_malfunction
    :precondition (robot_system_nominal)
    :effect (or (door_habitat_airlock_faulty) (door_airlock_surface_faulty))
  )

  (:action repair_door
    :precondition (or (robot_inside_habitat) (robot_inside_airlock))
    :effect (and (or (door_habitat_airlock_operational) (door_airlock_surface_operational))
                 (not (or (door_habitat_airlock_faulty) (door_airlock_surface_faulty))))
  )

  (:action pick_up_lunar_sample
    :precondition (and (robot_outside_habitat) (lunar_sample_on_surface) (robot_system_nominal))
    :effect (and (lunar_sample_with_robot) (not (lunar_sample_on_surface)))
  )

  (:action place_lunar_sample_in_habitat
    :precondition (and (robot_inside_habitat) (lunar_sample_with_robot) (robot_system_nominal))
    :effect (and (lunar_sample_in_habitat) (not (lunar_sample_with_robot)))
  )

  ;; ----- MODEL 3 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action perform_habitat_maintenance
    :precondition (and (robot_inside_habitat) (habitat_maintenance_required) (robot_system_nominal))
    :effect (and (habitat_maintenance_completed) (habitat_systems_nominal)
                 (not (habitat_maintenance_required))
                 (not (air_filter_fault)) (not (solar_panel_fault)) (not (temperature_control_fault)))
  )

  (:action assist_emergency_communication
    :precondition (and (emergency_communication_active) (robot_system_nominal))
    :effect (emergency_communication_active)
  )

  (:action recharge_battery
    :precondition (and (robot_power_low) (or (robot_inside_habitat) (robot_inside_airlock)))
    :effect (and (robot_power_charging) (not (robot_power_low)))
  )

  (:action complete_battery_recharge
    :precondition (robot_power_charging)
    :effect (and (robot_power_normal) (not (robot_power_charging)))
  )

  ;; ----- MODEL 4 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action astronaut_give_key_to_robot
    :precondition (and (key_with_astronaut)
                       (or (astronaut_inside_habitat) (astronaut_inside_airlock))
                       (or (robot_inside_habitat) (robot_inside_airlock)))
    :effect (and (key_with_robot) (not (key_with_astronaut)))
  )

  (:action resolve_emergency_communication
    :precondition (and (emergency_communication_active) (emergency_acknowledged_by_astronaut) (robot_system_nominal))
    :effect (and (emergency_resolved) (not (emergency_communication_active)) (not (emergency_acknowledged_by_astronaut)))
  )

  (:action detect_habitat_failure
    :precondition (and (robot_inside_habitat) (robot_system_nominal))
    :effect (or (air_filter_fault) (solar_panel_fault) (temperature_control_fault))
  )

  ;; ----- MODEL 5 ACTIONS -----

  (:action find_key
    :precondition (and (or (robot_inside_habitat) (robot_inside_airlock))
                       (or (key_with_astronaut) (key_in_habitat) (key_in_airlock) (backup_key_available)))
    :effect (and (key_with_robot)
                 (not (key_with_astronaut)) (not (key_in_habitat)) (not (key_in_airlock)) (not (backup_key_available)))
  )

  (:action acknowledge_emergency
    :precondition (and (emergency_communication_active)
                       (or (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)))
    :effect (emergency_acknowledged_by_astronaut)
  )

  (:action monitor_astronaut_health
    :precondition (and (or (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface))
                       (robot_system_nominal))
    :effect (or (astronaut_health_alert) (no_astronaut_health_alert))
  )

  (:action initiate_medical_response
    :precondition (and (astronaut_health_alert)
                       (or (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)))
    :effect (and (astronaut_medical_response_initiated) (not (astronaut_health_alert)))
  )

  (:action inspect_structural_integrity
    :precondition (and (structural_integrity_check_required)
                       (or (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat))
                       (robot_system_nominal))
    :effect (and (structural_integrity_check_completed) (not (structural_integrity_check_required)))
  )

  (:action inspect_environmental_factors
    :precondition (and (robot_outside_habitat) (robot_system_nominal))
    :effect (or (lunar_dust_contamination_detected) (no_lunar_dust_contamination)
                (temperature_variation_detected) (no_temperature_variation))
  )
)
