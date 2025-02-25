(define (domain space_mars_science_team)
  (:requirements :strips)

  (:predicates
    ;; ----- MODEL 0 PREDICATES -----
    (sample_detected)
    (robot_has_sample)
    (sample_analyzed)
    (findings_ready)

    ;; ----- MODEL 1 PREDICATES -----
    (robot_available)
    (robot_needs_recharge)
    (communication_delayed)
    (sample_type_identified)
    (sample_contaminated)
    (delayed_response)
    (task_synchronized)
    (environment_monitored)
    (resource_identified)
    (emergency_detected)

    ;; ----- MODEL 2 PREDICATES -----
    (robot_moving)
    (robot_stuck)
    (power_low)
    (mission_interrupted)
    (multi_robot_sync)
    (failure_reported)
    (soil_sample_collected)
    (atmospheric_data_collected)
    (infrastructure_inspected)
    (ground_control_override_active)

    ;; ----- MODEL 3 PREDICATES -----
    (weather_hazard_detected)
    (robot_damaged)
    (communication_blackout)
    (critical_system_failure)
    (equipment_calibrated)
    (maintenance_required)
    (long_term_data_stored)
    (redundant_communication_active)

    ;; ----- MODEL 4 PREDICATES -----
    (data_backup_created)
    (ground_control_ack_received)

    ;; ----- MODEL 5 PREDICATES -----
    (contamination_detected)
    (solar_panels_cleaned)
    (long_term_wear_detected)
    (diagnostic_health_check_completed)
  )

  ;; ----- MODEL 0 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action pick_up_sample
    :precondition (and robot_available sample_detected sample_type_identified (not robot_stuck) (not mission_interrupted))
    :effect (and (robot_has_sample) (not sample_detected))
  )

  (:action analyze_sample
    :precondition (and robot_available robot_has_sample (not robot_stuck) (not mission_interrupted))
    :effect (and (sample_analyzed) (findings_ready) (not robot_has_sample))
  )

  (:action report_findings
    :precondition (and findings_ready communication_delayed (not communication_blackout) ground_control_ack_received)
    :effect (and (delayed_response) (not findings_ready))
  )

  (:action drop_sample
    :precondition (and robot_has_sample (not mission_interrupted))
    :effect (not robot_has_sample)
  )

  ;; ----- MODEL 1 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action scan_for_samples
    :precondition (and robot_available (not robot_stuck) (not mission_interrupted))
    :effect (sample_detected)
  )

  (:action transmit_findings
    :precondition (and findings_ready communication_delayed (not communication_blackout) ground_control_ack_received)
    :effect (not delayed_response)
  )

  (:action coordinate_with_other_robots
    :precondition (and robot_available (not mission_interrupted) multi_robot_sync)
    :effect (task_synchronized)
  )

  (:action monitor_environment
    :precondition (and robot_available (not weather_hazard_detected) (not mission_interrupted))
    :effect (environment_monitored)
  )

  (:action identify_resources
    :precondition (and robot_available environment_monitored (not robot_stuck) (not mission_interrupted))
    :effect (resource_identified)
  )

  (:action respond_to_emergency
    :precondition (emergency_detected)
    :effect (not emergency_detected)
  )

  ;; ----- MODEL 2 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action navigate_to_sample
    :precondition (and robot_available (not robot_stuck) (not mission_interrupted) (not weather_hazard_detected))
    :effect (robot_moving)
  )

  (:action recharge_robot
    :precondition (and power_low (not robot_moving) (not mission_interrupted))
    :effect (and (robot_available) (not power_low))
  )

  (:action self_recover
    :precondition (and robot_stuck (not mission_interrupted))
    :effect (and (robot_available) (not robot_stuck))
  )

  (:action pause_mission
    :precondition (emergency_detected)
    :effect (mission_interrupted)
  )

  (:action sync_with_team
    :precondition (and task_synchronized (not mission_interrupted))
    :effect (multi_robot_sync)
  )

  (:action request_help_from_team
    :precondition (and robot_stuck multi_robot_sync)
    :effect (multi_robot_sync)
  )

  (:action report_failure_to_ground
    :precondition (and emergency_detected communication_delayed (not communication_blackout))
    :effect (failure_reported)
  )

  (:action collect_soil_sample
    :precondition (and robot_available (not robot_stuck))
    :effect (soil_sample_collected)
  )

  (:action collect_atmospheric_data
    :precondition (and robot_available (not robot_stuck))
    :effect (atmospheric_data_collected)
  )

  (:action inspect_infrastructure
    :precondition (and robot_available (not mission_interrupted))
    :effect (infrastructure_inspected)
  )

  (:action engage_ground_control_override
    :precondition (and robot_available (not mission_interrupted))
    :effect (ground_control_override_active)
  )

  (:action disengage_ground_control_override
    :precondition (ground_control_override_active)
    :effect (not ground_control_override_active)
  )

  ;; ----- MODEL 3 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action detect_weather_hazard
    :precondition (robot_available)
    :effect (weather_hazard_detected)
  )

  (:action assess_damage
    :precondition (and robot_available robot_damaged (not mission_interrupted))
    :effect (failure_reported)
  )

  (:action attempt_autonomous_repair
    :precondition (and robot_damaged (not critical_system_failure))
    :effect (and (robot_available) (not robot_damaged))
  )

  (:action safe_mode_activation
    :precondition (critical_system_failure)
    :effect (mission_interrupted)
  )

  (:action calibrate_equipment
    :precondition (and robot_available (not mission_interrupted))
    :effect (equipment_calibrated)
  )

  (:action perform_maintenance
    :precondition (and robot_available maintenance_required (not mission_interrupted))
    :effect (not maintenance_required)
  )

  (:action store_long_term_data
    :precondition (and findings_ready (not communication_blackout) data_backup_created)
    :effect (long_term_data_stored)
  )

  (:action activate_redundant_communication
    :precondition (and communication_blackout (not redundant_communication_active))
    :effect (redundant_communication_active)
  )

  ;; ----- MODEL 4 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action create_data_backup
    :precondition (and findings_ready (not data_backup_created))
    :effect (data_backup_created)
  )

  (:action receive_ground_control_ack
    :precondition (and delayed_response (not communication_blackout))
    :effect (and (ground_control_ack_received) (not delayed_response))
  )

  ;; ----- MODEL 5 ACTIONS -----

  (:action perform_diagnostic_health_check
    :precondition (and robot_available (not mission_interrupted))
    :effect (diagnostic_health_check_completed)
  )

  (:action detect_contamination
    :precondition (and robot_available (not mission_interrupted))
    :effect (contamination_detected)
  )

  (:action clean_solar_panels
    :precondition (and robot_available (not mission_interrupted))
    :effect (solar_panels_cleaned)
  )

  (:action assess_long_term_wear
    :precondition (and robot_available (not mission_interrupted))
    :effect (long_term_wear_detected)
  )

  (:action report_status_to_ground_control
    :precondition (and robot_available (not communication_blackout))
    :effect (ground_control_ack_received)
  )
)
