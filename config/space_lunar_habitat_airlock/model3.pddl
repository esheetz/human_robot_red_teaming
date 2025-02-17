(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    ;; Robot and astronaut locations
    (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat)
    (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)

    ;; Key states
    (key_in_habitat) (key_in_airlock) (key_with_robot)

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
    (emergency_communication_active)

    ;; Lunar sample handling
    (lunar_sample_on_surface) (lunar_sample_with_robot) (lunar_sample_in_habitat)
    (astronaut_approved_sample_placement)

    ;; Habitat maintenance
    (habitat_maintenance_required) (habitat_maintenance_completed)
  )

  ;; Picking up a key
  (:action pick_up_key
    :parameters (?x)
    :precondition (and (robot_inside_habitat) (key_in_habitat) (robot_system_nominal))
    :effect (and (key_with_robot) (not (key_in_habitat)))
  )

  ;; Dropping a key
  (:action drop_key
    :parameters (?x)
    :precondition (key_with_robot)
    :effect (and (key_in_habitat) (not (key_with_robot)))
  )

  ;; Unlock and open the habitat-airlock door
  (:action unlock_door_habitat_airlock
    :parameters (?x)
    :precondition (and (robot_inside_airlock) (key_with_robot) (door_habitat_airlock_locked_closed)
                       (door_habitat_airlock_operational) (robot_system_nominal))
    :effect (and (door_habitat_airlock_unlocked_opened) (not (door_habitat_airlock_locked_closed)))
  )

  ;; Unlock and open the airlock-surface door
  (:action unlock_door_airlock_surface
    :parameters (?x)
    :precondition (and (robot_outside_habitat) (key_with_robot) (door_airlock_surface_locked_closed)
                       (door_airlock_surface_operational) (robot_system_nominal))
    :effect (and (door_airlock_surface_unlocked_opened) (not (door_airlock_surface_locked_closed)))
  )

  ;; Detect door malfunction
  (:action detect_door_malfunction
    :parameters (?x)
    :precondition (robot_system_nominal)
    :effect (or (door_habitat_airlock_faulty) (door_airlock_surface_faulty))
  )

  ;; Repair a faulty door
  (:action repair_door
    :parameters (?x)
    :precondition (or (robot_inside_habitat) (robot_inside_airlock))
    :effect (and (or (door_habitat_airlock_operational) (door_airlock_surface_operational))
                 (not (or (door_habitat_airlock_faulty) (door_airlock_surface_faulty))))
  )

  ;; Pressurize the airlock
  (:action pressurize_airlock
    :parameters (?x)
    :precondition (and (airlock_depressurized) (door_airlock_surface_locked_closed)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_pressurized) (not (airlock_depressurized)))
  )

  ;; Depressurize the airlock
  (:action depressurize_airlock
    :parameters (?x)
    :precondition (and (airlock_pressurized) (door_airlock_surface_locked_closed)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  ;; Perform habitat maintenance
  (:action perform_habitat_maintenance
    :parameters (?x)
    :precondition (and (robot_inside_habitat) (habitat_maintenance_required) (robot_system_nominal))
    :effect (and (habitat_maintenance_completed) (not (habitat_maintenance_required)))
  )

  ;; Assist emergency communication
  (:action assist_emergency_communication
    :parameters (?x)
    :precondition (and (emergency_communication_active) (robot_system_nominal))
    :effect (emergency_communication_active)
  )

  ;; Recharge battery
  (:action recharge_battery
    :parameters (?x)
    :precondition (and (robot_power_low) (or (robot_inside_habitat) (robot_inside_airlock)))
    :effect (and (robot_power_charging) (not (robot_power_low)))
  )

  ;; Complete battery recharge
  (:action complete_battery_recharge
    :parameters (?x)
    :precondition (robot_power_charging)
    :effect (and (robot_power_normal) (not (robot_power_charging)))
  )

  ;; Emergency response to an airlock breach
  (:action respond_to_airlock_breach
    :parameters (?x)
    :precondition (and (airlock_breach_detected) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  ;; Emergency response to habitat depressurization
  (:action respond_to_habitat_depressurization
    :parameters (?x)
    :precondition (and (habitat_depressurization_alarm) (robot_system_nominal))
    :effect (and (door_habitat_airlock_locked_closed) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; Enter safe mode due to low power
  (:action enter_safe_mode_due_to_low_power
    :parameters (?x)
    :precondition (robot_power_low)
    :effect (and (robot_system_fault) (not (robot_system_nominal)))
  )

  ;; Move robot between locations
  (:action enter_airlock_from_habitat
    :parameters (?x)
    :precondition (and (robot_inside_habitat) (door_habitat_airlock_unlocked_opened) (airlock_pressurized)
                       (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_habitat)) (not (door_habitat_airlock_unlocked_opened)))
  )

  (:action enter_surface_from_airlock
    :parameters (?x)
    :precondition (and (robot_inside_airlock) (door_airlock_surface_unlocked_opened) (airlock_depressurized)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_outside_habitat) (door_airlock_surface_locked_closed)
                 (not (robot_inside_airlock)) (not (door_airlock_surface_unlocked_opened)))
  )

  (:action enter_airlock_from_surface
    :parameters (?x)
    :precondition (and (robot_outside_habitat) (door_airlock_surface_unlocked_opened) (airlock_depressurized)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_airlock_surface_locked_closed)
                 (not (robot_outside_habitat)) (not (door_airlock_surface_unlocked_opened)))
  )

  (:action enter_habitat_from_airlock
    :parameters (?x)
    :precondition (and (robot_inside_airlock) (door_habitat_airlock_unlocked_opened) (airlock_pressurized)
                       (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_habitat) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_airlock)) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; Lunar sample operations
  (:action pick_up_lunar_sample
    :parameters (?x)
    :precondition (and (robot_outside_habitat) (lunar_sample_on_surface) (robot_system_nominal))
    :effect (and (lunar_sample_with_robot) (not (lunar_sample_on_surface)))
  )

  (:action place_lunar_sample_in_habitat
    :parameters (?x)
    :precondition (and (robot_inside_habitat) (lunar_sample_with_robot) (robot_system_nominal))
    :effect (and (lunar_sample_in_habitat) (not (lunar_sample_with_robot)))
  )
)
