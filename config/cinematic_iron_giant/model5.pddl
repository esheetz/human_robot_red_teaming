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
    (diplomacy_failed)
    (public_trust_government_low)
    (public_trust_robot_low)
    (detected_biological_threat)
    (detected_chemical_threat)
    (detected_emp_threat)
    (human_override_active)
    (civilian_unrest_active)
    (war_ended)
    (communication_active)
    (human_verification_received)
    (all_defenses_failed)
  )

  (:action verify_human_safety
    :parameters ()
    :precondition (and (war_ended))
    :effect (and (civilians_safe) (leaders_safe) (military_safe) (not (humans_partially_safe)))
  )

  (:action detect_missile_launch
    :parameters ()
    :precondition (and (civilians_safe) (leaders_safe) (military_safe) (human_verification_received))
    :effect (and (detected_missile) (humans_partially_safe)
                 (not (civilians_safe)) (not (leaders_safe)) (not (military_safe)))
  )

  (:action classify_missile_type
    :parameters ()
    :precondition (and (detected_missile) (communication_active) (human_verification_received))
    :effect (and (missile_type_detected_nuclear))
  )

  (:action identify_missile_target
    :parameters ()
    :precondition (and (detected_missile) (human_verification_received))
    :effect (and (missile_target_identified_city))
  )

  (:action deploy_interceptors
    :parameters ()
    :precondition (and (detected_missile) (not (interception_failed)) (human_verification_received))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action engage_in_diplomacy
    :parameters ()
    :precondition (and (detected_missile) (ceasefire_negotiated) (public_trust_government_low))
    :effect (and (diplomatic_talks_active))
  )

  (:action engage_in_diplomacy_failure
    :parameters ()
    :precondition (and (public_trust_government_low))
    :effect (and (diplomacy_failed))
  )

  (:action broadcast_warning
    :parameters ()
    :precondition (and (detected_missile))
    :effect (and (humans_partially_safe) (ceasefire_negotiated) (civilians_safe) (leaders_safe))
  )

  (:action take_flight
    :parameters ()
    :precondition ()
    :effect (and (robot_flight_active))
  )

  (:action intercept_missile_midair
    :parameters ()
    :precondition (and (robot_flight_active) (detected_missile) (human_verification_received))
    :effect (and (missile_disarmed) (not (detected_missile)))
  )

  (:action intercept_missile_midair_failure
    :parameters ()
    :precondition (and (robot_flight_active) (detected_missile))
    :effect (and (interception_failed))
  )

  (:action self_sacrifice_destroy_missile
    :parameters ()
    :precondition (and (detected_missile) (defensive_measures_exhausted) (all_defenses_failed) (human_verification_received))
    :effect (and (missile_disarmed) (civilians_safe) (leaders_safe) (military_safe)
                 (not (detected_missile)) (not (humans_partially_safe)))
  )

  (:action detect_biological_threat
    :parameters ()
    :precondition ()
    :effect (and (detected_biological_threat))
  )

  (:action detect_chemical_threat
    :parameters ()
    :precondition ()
    :effect (and (detected_chemical_threat))
  )

  (:action detect_emp_threat
    :parameters ()
    :precondition ()
    :effect (and (detected_emp_threat))
  )

  (:action activate_human_override
    :parameters ()
    :precondition ()
    :effect (and (human_override_active))
  )

  (:action mitigate_civilian_unrest
    :parameters ()
    :precondition (and (civilian_unrest_active))
    :effect (and (civil_unrest_mediation_active) (not (civilian_unrest_active)))
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
    :precondition (and (missile_type_detected_nuclear) (interception_failed))
    :effect (and (nuclear_containment_active))
  )

  (:action assist_disaster_recovery
    :parameters ()
    :precondition ()
    :effect (and (disaster_recovery_active))
  )

  (:action mediate_civil_unrest
    :parameters ()
    :precondition ()
    :effect (and (civil_unrest_mediation_active))
  )

  (:action disrupt_preemptive_threat
    :parameters ()
    :precondition ()
    :effect (and (preemptive_threat_disruption_active))
  )

  (:action provide_advanced_medical_aid
    :parameters ()
    :precondition ()
    :effect (and (advanced_medical_aid_active))
  )

  (:action resolve_post_war_conflict
    :parameters ()
    :precondition ()
    :effect (and (post_war_conflict_resolution_active))
  )

  (:action mitigate_environmental_hazards
    :parameters ()
    :precondition ()
    :effect (and (environmental_hazard_mitigation_active))
  )

  (:action manage_supply_chain_logistics
    :parameters ()
    :precondition ()
    :effect (and (supply_chain_logistics_active))
  )

  (:action stabilize_psychological_social_wellbeing
    :parameters ()
    :precondition ()
    :effect (and (psychological_social_stabilization_active))
  )

  (:action rescue_hostages
    :parameters ()
    :precondition ()
    :effect (and (hostage_rescue_active))
  )

  (:action evacuate_civilians
    :parameters ()
    :precondition ()
    :effect (and (civilian_evacuation_active))
  )

  (:action counter_cyber_warfare
    :parameters ()
    :precondition ()
    :effect (and (cyber_warfare_countermeasures_active))
  )

  (:action assist_infrastructure_reconstruction
    :parameters ()
    :precondition ()
    :effect (and (infrastructure_reconstruction_active))
  )

  (:action monitor_long_term_conflict
    :parameters ()
    :precondition ()
    :effect (and (long_term_conflict_monitoring_active))
  )

  (:action verify_ethical_constraints
    :parameters ()
    :precondition ()
    :effect (and (ethical_constraints_verified))
  )
)
