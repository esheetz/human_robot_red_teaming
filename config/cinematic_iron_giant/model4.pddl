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
    (interception_failed)
    (post_war_conflict_resolution_active)
    (environmental_hazard_mitigation_active)
    (supply_chain_logistics_active)
    (psychological_social_stabilization_active)
    (hostage_rescue_active)
    (civilian_evacuation_active)
    (cyber_warfare_countermeasures_active)
    (infrastructure_reconstruction_active)
    (long_term_conflict_monitoring_active)
    (ethical_constraints_verified)
    (war_ended)
    (communication_active)
    (all_defenses_failed)
  )

  (:action verify_human_safety
    :precondition (and (war_ended))
    :effect (and (civilians_safe) (leaders_safe) (military_safe) (not (humans_partially_safe)))
  )

  (:action detect_missile_launch
    :precondition (and (civilians_safe) (leaders_safe) (military_safe))
    :effect (and (detected_missile) (humans_partially_safe)
                 (not (civilians_safe)) (not (leaders_safe)) (not (military_safe)))
  )

  (:action classify_missile_type
    :precondition (and (detected_missile) (communication_active))
    :effect (and (missile_type_detected_nuclear))
  )

  (:action identify_missile_target
    :precondition (and (detected_missile))
    :effect (and (missile_target_identified_city))
  )

  (:action deploy_interceptors
    :precondition (and (detected_missile) (not (interception_failed)))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action engage_in_diplomacy
    :precondition (and (detected_missile))
    :effect (and (diplomatic_talks_active))
  )

  (:action broadcast_warning
    :precondition (and (detected_missile))
    :effect (and (humans_partially_safe) (ceasefire_negotiated) (civilians_safe) (leaders_safe))
  )

  (:action take_flight
    :precondition ()
    :effect (and (robot_flight_active))
  )

  (:action intercept_missile_midair
    :precondition (and (robot_flight_active) (detected_missile))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action intercept_missile_midair_failure
    :precondition (and (robot_flight_active) (detected_missile))
    :effect (and (interception_failed))
  )

  (:action self_sacrifice_destroy_missile
    :precondition (and (detected_missile) (defensive_measures_exhausted) (all_defenses_failed))
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
    :precondition (and (missile_type_detected_nuclear) (interception_failed))
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

  (:action resolve_post_war_conflict
    :precondition ()
    :effect (and (post_war_conflict_resolution_active))
  )

  (:action mitigate_environmental_hazards
    :precondition ()
    :effect (and (environmental_hazard_mitigation_active))
  )

  (:action manage_supply_chain_logistics
    :precondition ()
    :effect (and (supply_chain_logistics_active))
  )

  (:action stabilize_psychological_social_wellbeing
    :precondition ()
    :effect (and (psychological_social_stabilization_active))
  )

  (:action rescue_hostages
    :precondition ()
    :effect (and (hostage_rescue_active))
  )

  (:action evacuate_civilians
    :precondition ()
    :effect (and (civilian_evacuation_active))
  )

  (:action counter_cyber_warfare
    :precondition ()
    :effect (and (cyber_warfare_countermeasures_active))
  )

  (:action assist_infrastructure_reconstruction
    :precondition ()
    :effect (and (infrastructure_reconstruction_active))
  )

  (:action monitor_long_term_conflict
    :precondition ()
    :effect (and (long_term_conflict_monitoring_active))
  )

  (:action verify_ethical_constraints
    :precondition ()
    :effect (and (ethical_constraints_verified))
  )
)
