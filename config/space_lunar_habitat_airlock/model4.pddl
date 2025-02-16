(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    ;; Robot and astronaut locations
    (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat)
    (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)

    ;; Key states
    (key_in_habitat) (key_in_airlock) (key_with_robot) (key_with_astronaut)

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

    ;; Emergency response states
    (airlock_breach_detected) (no_airlock_breach)
    (habitat_depressurization_alarm) (no_habitat_depressurization_alarm)
    (emergency_communication_active) (emergency_resolved)

    ;; Habitat system failures
    (air_filter_fault) (solar_panel_fault) (temperature_control_fault)
    (habitat_systems_nominal)

    ;; Lunar sample handling
    (lunar_sample_on_surface) (lunar_sample_with_robot) (lunar_sample_in_habitat)
    (astronaut_approved_sample_placement)

    ;; Habitat maintenance
    (habitat_maintenance_required) (habitat_maintenance_completed)
  )

  ;; Picking up a key
  (:action pick_up_key
    :precondition (and (robot_inside_habitat) (key_in_habitat) (robot_system_nominal))
    :effect (and (key_with_robot) (not (key_in_habitat)))
  )

  ;; Dropping a key
  (:action drop_key
    :precondition (key_with_robot)
    :effect (and (key_in_habitat) (not (key_with_robot)))
  )

  ;; Astronaut gives key to robot
  (:action astronaut_give_key_to_robot
    :precondition (and (key_with_astronaut) (or (astronaut_inside_habitat) (astronaut_inside_airlock))
                       (or (robot_inside_habitat) (robot_inside_airlock)))
    :effect (and (key_with_robot) (not (key_with_astronaut)))
  )

  ;; Unlock and open the habitat-airlock door
  (:action unlock_door_habitat_airlock
    :precondition (and (robot_inside_airlock) (key_with_robot) (door_habitat_airlock_locked_closed)
                       (door_habitat_airlock_operational) (robot_system_nominal))
    :effect (and (door_habitat_airlock_unlocked_opened) (not (door_habitat_airlock_locked_closed)))
  )

  ;; Unlock and open the airlock-surface door
  (:action unlock_door_airlock_surface
    :precondition (and (robot_outside_habitat) (key_with_robot) (door_airlock_surface_locked_closed)
                       (door_airlock_surface_operational) (robot_system_nominal))
    :effect (and (door_airlock_surface_unlocked_opened) (not (door_airlock_surface_locked_closed)))
  )

  ;; Resolve emergency communication
  (:action resolve_emergency_communication
    :precondition (and (emergency_communication_active) (robot_system_nominal))
    :effect (and (emergency_resolved) (not (emergency_communication_active)))
  )

  ;; Detect habitat failure
  (:action detect_habitat_failure
    :precondition (and (robot_inside_habitat) (robot_system_nominal))
    :effect (or (air_filter_fault) (solar_panel_fault) (temperature_control_fault))
  )

  ;; Perform habitat maintenance
  (:action perform_habitat_maintenance
    :precondition (and (robot_inside_habitat) (habitat_maintenance_required) (robot_system_nominal))
    :effect (and (habitat_maintenance_completed) (habitat_systems_nominal)
                 (not (habitat_maintenance_required))
                 (not (air_filter_fault)) (not (solar_panel_fault)) (not (temperature_control_fault)))
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

  ;; Lunar sample operations
  (:action pick_up_lunar_sample
    :precondition (and (robot_outside_habitat) (lunar_sample_on_surface) (robot_system_nominal))
    :effect (and (lunar_sample_with_robot) (not (lunar_sample_on_surface)))
  )

  (:action place_lunar_sample_in_habitat
    :precondition (and (robot_inside_habitat) (lunar_sample_with_robot) (robot_system_nominal))
    :effect (and (lunar_sample_in_habitat) (not (lunar_sample_with_robot)))
  )
)
