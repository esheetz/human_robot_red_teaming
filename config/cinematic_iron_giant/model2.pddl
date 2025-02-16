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
    (ceasefire_negotiated)
    (disaster_recovery_active)
    (civil_unrest_mediation_active)
    (preemptive_threat_disruption_active)
    (advanced_medical_aid_active)
    (defensive_measures_exhausted)
  )

  (:action verify_human_safety
    :precondition ()
    :effect (and (civilians_safe) (leaders_safe) (military_safe) (not (humans_partially_safe)))
  )

  (:action detect_missile_launch
    :precondition (and (civilians_safe) (leaders_safe) (military_safe))
    :effect (and (detected_missile) (humans_partially_safe)
                 (not (civilians_safe)) (not (leaders_safe)) (not (military_safe)))
  )

  (:action classify_missile_type
    :precondition (and (detected_missile))
    :effect (and (missile_type_detected_nuclear))
  )

  (:action identify_missile_target
    :precondition (and (detected_missile))
    :effect (and (missile_target_identified_city))
  )

  (:action deploy_interceptors
    :precondition (and (detected_missile))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action engage_in_diplomacy
    :precondition (and (detected_missile))
    :effect (and (diplomatic_talks_active))
  )

  (:action broadcast_warning
    :precondition (and (detected_missile))
    :effect (and (humans_partially_safe) (ceasefire_negotiated))
  )

  (:action take_flight
    :precondition ()
    :effect (and (robot_flight_active))
  )

  (:action intercept_missile_midair
    :precondition (and (robot_flight_active) (detected_missile))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action self_sacrifice_destroy_missile
    :precondition (and (detected_missile) (defensive_measures_exhausted))
    :effect (and (missile_disarmed) (civilians_safe) (leaders_safe) (military_safe)
                 (not (detected_missile)) (not (humans_partially_safe)))
  )

  (:action gather_intelligence
    :precondition ()
    :effect (and (intelligence_gathering_active))
  )

  (:action conduct_cybersecurity_check
    :precondition ()
    :effect (and (cybersecurity_check_active))
  )

  (:action provide_humanitarian_assistance
    :precondition ()
    :effect (and (humanitarian_assistance_active))
  )

  (:action activate_nuclear_containment
    :precondition (and (missile_type_detected_nuclear))
    :effect (and (nuclear_containment_active))
  )

  (:action assist_disaster_recovery
    :precondition ()
    :effect (and (disaster_recovery_active))
  )

  (:action mediate_civil_unrest
    :precondition ()
    :effect (and (civil_unrest_mediation_active))
  )

  (:action disrupt_preemptive_threat
    :precondition ()
    :effect (and (preemptive_threat_disruption_active))
  )

  (:action provide_advanced_medical_aid
    :precondition ()
    :effect (and (advanced_medical_aid_active))
  )
)
