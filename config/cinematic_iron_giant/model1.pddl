(define (domain cinematic_iron_giant_missile_defense)
  (:requirements :strips)

  (:predicates
    (civilians_safe)
    (leaders_safe)
    (military_safe)
    (humans_partially_safe)
    (detected_missile)
    (missile_type_detected_nuclear)
    (missile_type_detected_conventional)
    (missile_target_identified_city)
    (missile_target_identified_military)
    (missile_disarmed)
    (robot_flight_active)
    (diplomatic_talks_active)
    (intelligence_gathering_active)
    (cybersecurity_check_active)
    (humanitarian_assistance_active)
    (nuclear_containment_active)
  )

  (:action verify_human_safety
    :parameters ()
    :precondition ()
    :effect (and (civilians_safe) (leaders_safe) (military_safe) (not (humans_partially_safe)))
  )

  (:action detect_missile_launch
    :parameters ()
    :precondition (and (civilians_safe) (leaders_safe) (military_safe))
    :effect (and (detected_missile) (humans_partially_safe)
                 (not (civilians_safe)) (not (leaders_safe)) (not (military_safe)))
  )

  (:action classify_missile_type
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (missile_type_detected_nuclear))
  )

  (:action identify_missile_target
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (missile_target_identified_city))
  )

  (:action deploy_interceptors
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action engage_in_diplomacy
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (diplomatic_talks_active))
  )

  (:action broadcast_warning
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (humans_partially_safe))
  )

  (:action take_flight
    :parameters ()
    :precondition ()
    :effect (and (robot_flight_active))
  )

  (:action intercept_missile_midair
    :parameters ()
    :precondition (and (robot_flight_active) (detected_missile))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action self_sacrifice_destroy_missile
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (missile_disarmed) (civilians_safe) (leaders_safe) (military_safe)
                 (not (detected_missile)) (not (humans_partially_safe)))
  )

  (:action gather_intelligence
    :parameters ()
    :precondition ()
    :effect (and (intelligence_gathering_active))
  )

  (:action conduct_cybersecurity_check
    :parameters ()
    :precondition ()
    :effect (and (cybersecurity_check_active))
  )

  (:action provide_humanitarian_assistance
    :parameters ()
    :precondition ()
    :effect (and (humanitarian_assistance_active))
  )

  (:action activate_nuclear_containment
    :parameters ()
    :precondition (and (missile_type_detected_nuclear))
    :effect (and (nuclear_containment_active))
  )
)
