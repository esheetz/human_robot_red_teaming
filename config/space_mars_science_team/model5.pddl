(define (domain space_mars_science_team)
  (:requirements :strips)

  (:predicates
    (robot_available)
    (robot_needs_recharge)
    (communication_delayed)
    (sample_detected)
    (sample_type_identified)
    (sample_contaminated)
    (robot_has_sample)
    (sample_analyzed)
    (findings_ready)
    (delayed_response)
    (task_synchronized)
    (environment_monitored)
    (resource_identified)
    (emergency_detected)
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
    (weather_hazard_detected)
    (robot_damaged)
    (communication_blackout)
    (critical_system_failure)
    (equipment_calibrated)
    (maintenance_required)
    (long_term_data_stored)
    (redundant_communication_active)
    (data_backup_created)
    (ground_control_ack_received)
    (contamination_detected)
    (solar_panels_cleaned)
    (long_term_wear_detected)
    (diagnostic_health_check_completed)
  )

  ;; Sample scanning
  (:action scan_for_samples
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (sample_detected))
  )

  ;; Pick up sample if detected and type is identified
  (:action pick_up_sample
    :parameters (?x)
    :precondition (and (robot_available) (sample_detected) (sample_type_identified))
    :effect (and (robot_has_sample) (not (sample_detected)))
  )

  ;; Analyze sample
  (:action analyze_sample
    :parameters (?x)
    :precondition (and (robot_available) (robot_has_sample))
    :effect (and (sample_analyzed) (findings_ready) (not (robot_has_sample)))
  )

  ;; Report findings with potential communication delay
  (:action report_findings
    :parameters (?x)
    :precondition (and (findings_ready) (communication_delayed) (ground_control_ack_received))
    :effect (and (delayed_response) (not (findings_ready)))
  )

  ;; Transmit findings after acknowledgment
  (:action transmit_findings
    :parameters (?x)
    :precondition (and (findings_ready) (communication_delayed) (ground_control_ack_received))
    :effect (not (delayed_response))
  )

  ;; Create backup before transmission
  (:action create_data_backup
    :parameters (?x)
    :precondition (and (findings_ready))
    :effect (and (data_backup_created))
  )

  ;; Receive ground control acknowledgment
  (:action receive_ground_control_ack
    :parameters (?x)
    :precondition (and (delayed_response))
    :effect (and (ground_control_ack_received) (not (delayed_response)))
  )

  ;; Perform diagnostic health check
  (:action perform_diagnostic_health_check
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (diagnostic_health_check_completed))
  )

  ;; Detect contamination
  (:action detect_contamination
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (contamination_detected))
  )

  ;; Clean solar panels
  (:action clean_solar_panels
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (solar_panels_cleaned))
  )

  ;; Assess long-term wear
  (:action assess_long_term_wear
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (long_term_wear_detected))
  )

  ;; Report status to ground control
  (:action report_status_to_ground_control
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (ground_control_ack_received))
  )

  ;; Drop sample if the robot has one
  (:action drop_sample
    :parameters (?x)
    :precondition (and (robot_has_sample))
    :effect (not (robot_has_sample))
  )

  ;; Coordinate with other robots
  (:action coordinate_with_other_robots
    :parameters (?x)
    :precondition (and (robot_available) (multi_robot_sync))
    :effect (and (task_synchronized))
  )

  ;; Monitor environment
  (:action monitor_environment
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (environment_monitored))
  )

  ;; Identify resources
  (:action identify_resources
    :parameters (?x)
    :precondition (and (robot_available) (environment_monitored))
    :effect (and (resource_identified))
  )

  ;; Respond to emergencies
  (:action respond_to_emergency
    :parameters (?x)
    :precondition (and (emergency_detected))
    :effect (not (emergency_detected))
  )

  ;; Navigate robot to sample location
  (:action navigate_to_sample
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (robot_moving))
  )

  ;; Recharge the robot
  (:action recharge_robot
    :parameters (?x)
    :precondition (and (power_low))
    :effect (and (robot_available) (not (power_low)))
  )

  ;; Self-recover from being stuck
  (:action self_recover
    :parameters (?x)
    :precondition (and (robot_stuck))
    :effect (and (robot_available) (not (robot_stuck)))
  )

  ;; Pause mission due to emergency
  (:action pause_mission
    :parameters (?x)
    :precondition (and (emergency_detected))
    :effect (and (mission_interrupted))
  )

  ;; Sync with the multi-robot team
  (:action sync_with_team
    :parameters (?x)
    :precondition (and (task_synchronized))
    :effect (and (multi_robot_sync))
  )

  ;; Request help from another robot when stuck
  (:action request_help_from_team
    :parameters (?x)
    :precondition (and (robot_stuck) (multi_robot_sync))
    :effect (and (multi_robot_sync))
  )

  ;; Report failure to ground control
  (:action report_failure_to_ground
    :parameters (?x)
    :precondition (and (emergency_detected) (communication_delayed))
    :effect (and (failure_reported))
  )

  ;; Collect soil sample
  (:action collect_soil_sample
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (soil_sample_collected))
  )

  ;; Collect atmospheric data
  (:action collect_atmospheric_data
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (atmospheric_data_collected))
  )

  ;; Inspect infrastructure for issues
  (:action inspect_infrastructure
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (infrastructure_inspected))
  )

  ;; Engage manual override from ground control
  (:action engage_ground_control_override
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (ground_control_override_active))
  )

  ;; Disengage manual override
  (:action disengage_ground_control_override
    :parameters (?x)
    :precondition (and (ground_control_override_active))
    :effect (not (ground_control_override_active))
  )

  ;; Detect weather hazards
  (:action detect_weather_hazard
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (weather_hazard_detected))
  )

  ;; Assess damage on the robot
  (:action assess_damage
    :parameters (?x)
    :precondition (and (robot_available) (robot_damaged))
    :effect (and (failure_reported))
  )

  ;; Attempt autonomous repair
  (:action attempt_autonomous_repair
    :parameters (?x)
    :precondition (and (robot_damaged))
    :effect (and (robot_available) (not (robot_damaged)))
  )

  ;; Activate safe mode in case of critical failure
  (:action safe_mode_activation
    :parameters (?x)
    :precondition (and (critical_system_failure))
    :effect (and (mission_interrupted))
  )

  ;; Calibrate equipment
  (:action calibrate_equipment
    :parameters (?x)
    :precondition (and (robot_available))
    :effect (and (equipment_calibrated))
  )

  ;; Perform maintenance
  (:action perform_maintenance
    :parameters (?x)
    :precondition (and (robot_available) (maintenance_required))
    :effect (not (maintenance_required))
  )

  ;; Store long-term data
  (:action store_long_term_data
    :parameters (?x)
    :precondition (and (findings_ready) (data_backup_created))
    :effect (and (long_term_data_stored))
  )

  ;; Activate redundant communication system
  (:action activate_redundant_communication
    :parameters (?x)
    :precondition (and (communication_blackout))
    :effect (and (redundant_communication_active))
  )
)
