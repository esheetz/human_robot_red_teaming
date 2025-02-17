(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    ;; Robot location states
    (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat)

    ;; Astronaut location states
    (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)

    ;; Key and access control
    (key_in_habitat) (key_in_airlock) (key_with_robot)

    ;; Door states
    (door_habitat_airlock_locked_closed) (door_habitat_airlock_unlocked_opened)
    (door_airlock_surface_locked_closed) (door_airlock_surface_unlocked_opened)

    ;; Door operational states
    (door_habitat_airlock_operational) (door_habitat_airlock_faulty)
    (door_airlock_surface_operational) (door_airlock_surface_faulty)

    ;; Airlock pressure states
    (airlock_pressurized) (airlock_depressurized)

    ;; Robot capabilities & status
    (robot_power_normal) (robot_power_low)
    (robot_system_nominal) (robot_system_fault)

    ;; Emergency response states
    (airlock_breach_detected) (no_airlock_breach)
    (habitat_depressurization_alarm) (no_habitat_depressurization_alarm)

    ;; Lunar sample operations
    (lunar_sample_on_surface) (lunar_sample_with_robot) (lunar_sample_in_habitat)
  )

  ;; Picking up a key
  (:action pick_up_key
    :parameters ()
    :precondition (and (robot_inside_habitat) (key_in_habitat) (robot_system_nominal))
    :effect (and (key_with_robot) (not (key_in_habitat)))
  )

  ;; Dropping a key
  (:action drop_key
    :parameters ()
    :precondition (key_with_robot)
    :effect (and (key_in_habitat) (not (key_with_robot)))
  )

  ;; Unlock and open the habitat-airlock door
  (:action unlock_door_habitat_airlock
    :parameters ()
    :precondition (and (robot_inside_airlock) (key_with_robot) (door_habitat_airlock_locked_closed)
                       (door_habitat_airlock_operational) (robot_system_nominal))
    :effect (and (door_habitat_airlock_unlocked_opened) (not (door_habitat_airlock_locked_closed)))
  )

  ;; Unlock and open the airlock-surface door
  (:action unlock_door_airlock_surface
    :parameters ()
    :precondition (and (robot_outside_habitat) (key_with_robot) (door_airlock_surface_locked_closed)
                       (door_airlock_surface_operational) (robot_system_nominal))
    :effect (and (door_airlock_surface_unlocked_opened) (not (door_airlock_surface_locked_closed)))
  )

  ;; Detect a malfunctioning door
  (:action detect_door_malfunction
    :parameters ()
    :precondition (robot_system_nominal)
    :effect (or (door_habitat_airlock_faulty) (door_airlock_surface_faulty))
  )

  ;; Repair a faulty door
  (:action repair_door
    :parameters ()
    :precondition (or (robot_inside_habitat) (robot_inside_airlock))
    :effect (and (or (door_habitat_airlock_operational) (door_airlock_surface_operational))
                 (not (or (door_habitat_airlock_faulty) (door_airlock_surface_faulty))))
  )

  ;; Pressurize the airlock
  (:action pressurize_airlock
    :parameters ()
    :precondition (and (airlock_depressurized) (door_airlock_surface_locked_closed)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_pressurized) (not (airlock_depressurized)))
  )

  ;; Depressurize the airlock
  (:action depressurize_airlock
    :parameters ()
    :precondition (and (airlock_pressurized) (door_airlock_surface_locked_closed)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  ;; Move the robot from the habitat into the airlock
  (:action enter_airlock_from_habitat
    :parameters ()
    :precondition (and (robot_inside_habitat) (door_habitat_airlock_unlocked_opened) (airlock_pressurized)
                       (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_habitat)) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; Move the robot from the airlock to the surface
  (:action enter_surface_from_airlock
    :parameters ()
    :precondition (and (robot_inside_airlock) (door_airlock_surface_unlocked_opened) (airlock_depressurized)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_outside_habitat) (door_airlock_surface_locked_closed)
                 (not (robot_inside_airlock)) (not (door_airlock_surface_unlocked_opened)))
  )

  ;; Move the robot from the surface into the airlock
  (:action enter_airlock_from_surface
    :parameters ()
    :precondition (and (robot_outside_habitat) (door_airlock_surface_unlocked_opened) (airlock_depressurized)
                       (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_airlock_surface_locked_closed)
                 (not (robot_outside_habitat)) (not (door_airlock_surface_unlocked_opened)))
  )

  ;; Move the robot from the airlock into the habitat
  (:action enter_habitat_from_airlock
    :parameters ()
    :precondition (and (robot_inside_airlock) (door_habitat_airlock_unlocked_opened) (airlock_pressurized)
                       (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_habitat) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_airlock)) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; Pick up a lunar sample from the surface
  (:action pick_up_lunar_sample
    :parameters ()
    :precondition (and (robot_outside_habitat) (lunar_sample_on_surface) (robot_system_nominal))
    :effect (and (lunar_sample_with_robot) (not (lunar_sample_on_surface)))
  )

  ;; Place the lunar sample inside the habitat
  (:action place_lunar_sample_in_habitat
    :parameters ()
    :precondition (and (robot_inside_habitat) (lunar_sample_with_robot) (robot_system_nominal))
    :effect (and (lunar_sample_in_habitat) (not (lunar_sample_with_robot)))
  )

  ;; Emergency response to an airlock breach
  (:action respond_to_airlock_breach
    :parameters ()
    :precondition (and (airlock_breach_detected) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  ;; Emergency response to habitat depressurization
  (:action respond_to_habitat_depressurization
    :parameters ()
    :precondition (and (habitat_depressurization_alarm) (robot_system_nominal))
    :effect (and (door_habitat_airlock_locked_closed) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; If robot power is low, enter safe mode
  (:action enter_safe_mode_due_to_low_power
    :parameters ()
    :precondition (robot_power_low)
    :effect (and (robot_system_fault) (not (robot_system_nominal)))
  )
)
