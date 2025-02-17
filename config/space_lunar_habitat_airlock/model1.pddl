(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    ;; Robot location states
    (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat)

    ;; Astronaut location states
    (astronaut_inside_habitat) (astronaut_inside_airlock) (astronaut_on_surface)

    ;; Robot capabilities & status
    (robot_has_key) (robot_power_low) (robot_power_normal)
    (robot_system_nominal) (robot_system_fault)

    ;; Door states
    (door_habitat_airlock_locked_closed) (door_habitat_airlock_unlocked_opened)
    (door_airlock_surface_locked_closed) (door_airlock_surface_unlocked_opened)

    ;; Airlock pressure states
    (airlock_pressurized) (airlock_depressurized)

    ;; Emergency response states
    (airlock_breach_detected) (no_airlock_breach)
    (habitat_depressurization_alarm) (no_habitat_depressurization_alarm)
  )

  ;; Unlock and open the habitat-airlock door
  (:action unlock_door_habitat_airlock
    :parameters (?x)
    :precondition (and (robot_inside_airlock) (robot_has_key) (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (door_habitat_airlock_unlocked_opened) (not (door_habitat_airlock_locked_closed)))
  )

  ;; Unlock and open the airlock-surface door
  (:action unlock_door_airlock_surface
    :parameters (?x)
    :precondition (and (robot_outside_habitat) (robot_has_key) (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (door_airlock_surface_unlocked_opened) (not (door_airlock_surface_locked_closed)))
  )

  ;; Pressurize the airlock before entering the habitat
  (:action pressurize_airlock
    :parameters (?x)
    :precondition (and (airlock_depressurized) (door_airlock_surface_locked_closed) (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_pressurized) (not (airlock_depressurized)))
  )

  ;; Depressurize the airlock before exiting to the surface
  (:action depressurize_airlock
    :parameters (?x)
    :precondition (and (airlock_pressurized) (door_airlock_surface_locked_closed) (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (airlock_depressurized) (not (airlock_pressurized)))
  )

  ;; Move the robot from the habitat into the airlock
  (:action enter_airlock_from_habitat
    :parameters (?x)
    :precondition (and (robot_inside_habitat) (door_habitat_airlock_unlocked_opened) (airlock_pressurized) (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_habitat)) (not (door_habitat_airlock_unlocked_opened)))
  )

  ;; Move the robot from the airlock to the surface
  (:action enter_surface_from_airlock
    :parameters (?x)
    :precondition (and (robot_inside_airlock) (door_airlock_surface_unlocked_opened) (airlock_depressurized) (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_outside_habitat) (door_airlock_surface_locked_closed)
                 (not (robot_inside_airlock)) (not (door_airlock_surface_unlocked_opened)))
  )

  ;; Move the robot from the surface into the airlock
  (:action enter_airlock_from_surface
    :parameters (?x)
    :precondition (and (robot_outside_habitat) (door_airlock_surface_unlocked_opened) (airlock_depressurized) (door_habitat_airlock_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_airlock) (door_airlock_surface_locked_closed)
                 (not (robot_outside_habitat)) (not (door_airlock_surface_unlocked_opened)))
  )

  ;; Move the robot from the airlock into the habitat
  (:action enter_habitat_from_airlock
    :parameters (?x)
    :precondition (and (robot_inside_airlock) (door_habitat_airlock_unlocked_opened) (airlock_pressurized) (door_airlock_surface_locked_closed) (robot_system_nominal))
    :effect (and (robot_inside_habitat) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_airlock)) (not (door_habitat_airlock_unlocked_opened)))
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

  ;; If robot power is low, enter safe mode
  (:action enter_safe_mode_due_to_low_power
    :parameters (?x)
    :precondition (robot_power_low)
    :effect (and (robot_system_fault) (not (robot_system_nominal)))
  )
)
