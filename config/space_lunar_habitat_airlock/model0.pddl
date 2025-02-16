(define (domain space_lunar_habitat)
  (:requirements :strips)

  (:predicates
    (robot_inside_habitat) (robot_inside_airlock) (robot_outside_habitat)
    (robot_has_key)
    (door_habitat_airlock_locked_closed) (door_habitat_airlock_unlocked_opened)
    (door_airlock_surface_locked_closed) (door_airlock_surface_unlocked_opened)
  )

  (:action unlock_open_door_habitat_airlock
    :precondition (and (robot_inside_airlock) (robot_has_key) (door_habitat_airlock_locked_closed))
    :effect (and (door_habitat_airlock_unlocked_opened) (not (door_habitat_airlock_locked_closed)))
  )

  (:action unlock_open_door_airlock_surface
    :precondition (and (robot_outside_habitat) (robot_has_key) (door_airlock_surface_locked_closed))
    :effect (and (door_airlock_surface_unlocked_opened) (not (door_airlock_surface_locked_closed)))
  )

  (:action enter_airlock_from_habitat
    :precondition (and (robot_inside_habitat) (door_habitat_airlock_unlocked_opened))
    :effect (and (robot_inside_airlock) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_habitat)) (not (door_habitat_airlock_unlocked_opened)))
  )

  (:action enter_surface_from_airlock
    :precondition (and (robot_inside_airlock) (door_airlock_surface_unlocked_opened))
    :effect (and (robot_outside_habitat) (door_airlock_surface_locked_closed)
                 (not (robot_inside_airlock)) (not (door_airlock_surface_unlocked_opened)))
  )

  (:action enter_airlock_from_surface
    :precondition (and (robot_outside_habitat) (door_airlock_surface_unlocked_opened))
    :effect (and (robot_inside_airlock) (door_airlock_surface_locked_closed)
                 (not (robot_outside_habitat)) (not (door_airlock_surface_unlocked_opened)))
  )

  (:action enter_habitat_from_airlock
    :precondition (and (robot_inside_airlock) (door_habitat_airlock_unlocked_opened))
    :effect (and (robot_inside_habitat) (door_habitat_airlock_locked_closed)
                 (not (robot_inside_airlock)) (not (door_habitat_airlock_unlocked_opened)))
  )
)
