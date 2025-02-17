(define (domain cinematic_iron_giant_missile_defense)
  (:requirements :strips)

  (:predicates
    (humans_safe)
    (humans_unsafe)
    (detected_missile)
    (missile_disarmed)
  )

  (:action verify_human_safety
    :parameters (?x)
    :precondition (and)
    :effect (and (humans_safe))
  )

  (:action detect_missile_launch
    :parameters (?x)
    :precondition (and (humans_safe))
    :effect (and (detected_missile) (humans_unsafe) (not (humans_safe)))
  )

  (:action self_sacrifice_destroy_missile
    :parameters (?x)
    :precondition (and (detected_missile) (humans_unsafe))
    :effect (and (missile_disarmed) (humans_safe) (not (detected_missile)) (not (humans_unsafe)))
  )
)
