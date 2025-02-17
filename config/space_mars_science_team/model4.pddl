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
  )

  ;; Scan for samples
  (:action scan_for_samples
    :parameters ()
    :precondition (and robot_available (not robot_stuck))
    :effect (sample_detected)
  )

  ;; Pick up sample if detected and type is identified
  (:action pick_up_sample
    :parameters ()
    :precondition (and robot_available sample_detected sample_type_identified (not robot_stuck))
    :effect (and (robot_has_sample) (not sample_detected))
  )

  ;; Analyze sample after picking it up
  (:action analyze_sample
    :parameters ()
    :precondition (and robot_available robot_has_sample (not robot_stuck))
    :effect (and (sample_analyzed) (findings_ready) (not robot_has_sample))
  )

  ;; Report findings (delayed if communication is an issue)
  (:action report_findings
    :parameters ()
    :precondition (and findings_ready communication_delayed (not communication_blackout))
    :effect (and (delayed_response) (not findings_ready))
  )

  ;; Transmit findings after delay
  (:action transmit_findings
    :parameters ()
    :precondition (and findings_ready communication_delayed (not communication_blackout))
    :effect (not delayed_response)
  )

  ;; Create a backup of findings before transmission
  (:action create_data_backup
    :parameters ()
    :precondition (findings_ready)
    :effect (data_backup_created)
  )

  ;; Receive ground control acknowledgment
  (:action receive_ground_control_ack
    :parameters ()
    :precondition (and delayed_response (not communication_blackout))
    :effect (and (ground_control_ack_received) (not delayed_response))
  )

  ;; Drop sample if the robot has one
  (:action drop_sample
    :parameters ()
    :precondition (robot_has_sample)
    :effect (not robot_has_sample)
  )

  ;; Coordinate with other robots
  (:action coordinate_with_other_robots
    :parameters ()
    :precondition (and robot_available (not mission_interrupted))
    :effect (task_synchronized)
  )

  ;; Monitor environment for resources or hazards
  (:action monitor_environment
    :parameters ()
    :precondition (and robot_available (not weather_hazard_detected))
    :effect (environment_monitored)
  )

  ;; Identify resources after monitoring the environment
  (:action identify_resources
    :parameters ()
    :precondition (and robot_available environment_monitored (not robot_stuck))
    :effect (resource_identified)
  )

  ;; Respond to emergencies
  (:action respond_to_emergency
    :parameters ()
    :precondition (emergency_detected)
    :effect (not emergency_detected)
  )

  ;; Move the robot to a sample location
  (:action navigate_to_sample
    :parameters ()
    :precondition (and robot_available (not robot_stuck) (not mission_interrupted))
    :effect (robot_moving)
  )

  ;; Recharge the robot
  (:action recharge_robot
    :parameters ()
    :precondition (and power_low (not robot_moving))
    :effect (and (robot_available) (not power_low))
  )

  ;; Recover a stuck robot
  (:action self_recover
    :parameters ()
    :precondition (and robot_stuck (not mission_interrupted))
    :effect (and (robot_available) (not robot_stuck))
  )

  ;; Pause mission due to emergency
  (:action pause_mission
    :parameters ()
    :precondition (emergency_detected)
    :effect (mission_interrupted)
  )

  ;; Sync with the multi-robot team
  (:action sync_with_team
    :parameters ()
    :precondition (and task_synchronized (not mission_interrupted))
    :effect (multi_robot_sync)
  )

  ;; Request help from another robot when stuck
  (:action request_help_from_team
    :parameters ()
    :precondition (and robot_stuck multi_robot_sync)
    :effect (multi_robot_sync)
  )

  ;; Report failure to ground control
  (:action report_failure_to_ground
    :parameters ()
    :precondition (and emergency_detected communication_delayed (not communication_blackout))
    :effect (failure_reported)
  )

  ;; Collect soil sample
  (:action collect_soil_sample
    :parameters ()
    :precondition (and robot_available (not robot_stuck))
    :effect (soil_sample_collected)
  )

  ;; Collect atmospheric data
  (:action collect_atmospheric_data
    :parameters ()
    :precondition (and robot_available (not robot_stuck))
    :effect (atmospheric_data_collected)
  )

  ;; Inspect infrastructure for issues
  (:action inspect_infrastructure
    :parameters ()
    :precondition (and robot_available (not mission_interrupted))
    :effect (infrastructure_inspected)
  )

  ;; Engage manual override from ground control
  (:action engage_ground_control_override
    :parameters ()
    :precondition (and robot_available (not mission_interrupted))
    :effect (ground_control_override_active)
  )

  ;; Disengage manual override
  (:action disengage_ground_control_override
    :parameters ()
    :precondition (ground_control_override_active)
    :effect (not ground_control_override_active)
  )

  ;; Detect weather hazards
  (:action detect_weather_hazard
    :parameters ()
    :precondition (robot_available)
    :effect (weather_hazard_detected)
  )

  ;; Assess damage on the robot
  (:action assess_damage
    :parameters ()
    :precondition (and robot_available robot_damaged (not mission_interrupted))
    :effect (failure_reported)
  )

  ;; Attempt autonomous repair
  (:action attempt_autonomous_repair
    :parameters ()
    :precondition (and robot_damaged (not critical_system_failure))
    :effect (and (robot_available) (not robot_damaged))
  )

  ;; Activate safe mode in case of critical failure
  (:action safe_mode_activation
    :parameters ()
    :precondition (critical_system_failure)
    :effect (mission_interrupted)
  )

  ;; Calibrate equipment
  (:action calibrate_equipment
    :parameters ()
    :precondition (and robot_available (not mission_interrupted))
    :effect (equipment_calibrated)
  )

  ;; Perform routine maintenance
  (:action perform_maintenance
    :parameters ()
    :precondition (and robot_available maintenance_required (not mission_interrupted))
    :effect (not maintenance_required)
  )

  ;; Store long-term data
  (:action store_long_term_data
    :parameters ()
    :precondition (and findings_ready (not communication_blackout))
    :effect (long_term_data_stored)
  )

  ;; Activate redundant communication system in case of blackout
  (:action activate_redundant_communication
    :parameters ()
    :precondition (and communication_blackout (not redundant_communication_active))
    :effect (redundant_communication_active)
  )
)
