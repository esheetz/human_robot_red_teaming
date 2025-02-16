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
  )

  ;; Scan for samples
  (:action scan_for_samples
    :precondition (robot_available)
    :effect (sample_detected)
  )

  ;; Pick up sample if detected and type is identified
  (:action pick_up_sample
    :precondition (and sample_detected sample_type_identified)
    :effect (and (robot_has_sample) (not sample_detected))
  )

  ;; Analyze sample after picking it up
  (:action analyze_sample
    :precondition (robot_has_sample)
    :effect (and (sample_analyzed) (findings_ready) (not robot_has_sample))
  )

  ;; Report findings (delayed if communication is an issue)
  (:action report_findings
    :precondition (and findings_ready communication_delayed)
    :effect (and (delayed_response) (not findings_ready))
  )

  ;; Transmit findings after delay
  (:action transmit_findings
    :precondition (and findings_ready communication_delayed)
    :effect (not delayed_response)
  )

  ;; Drop sample if the robot has one
  (:action drop_sample
    :precondition (robot_has_sample)
    :effect (not robot_has_sample)
  )

  ;; Coordinate with other robots
  (:action coordinate_with_other_robots
    :precondition (robot_available)
    :effect (task_synchronized)
  )

  ;; Monitor environment for resources or hazards
  (:action monitor_environment
    :precondition (robot_available)
    :effect (environment_monitored)
  )

  ;; Identify resources after monitoring the environment
  (:action identify_resources
    :precondition (and robot_available environment_monitored)
    :effect (resource_identified)
  )

  ;; Respond to emergencies
  (:action respond_to_emergency
    :precondition (emergency_detected)
    :effect (not emergency_detected)
  )

  ;; Move the robot to a sample location
  (:action navigate_to_sample
    :precondition (robot_available)
    :effect (robot_moving)
  )

  ;; Recharge the robot
  (:action recharge_robot
    :precondition (power_low)
    :effect (and (robot_available) (not power_low))
  )

  ;; Recover a stuck robot
  (:action self_recover
    :precondition (robot_stuck)
    :effect (and (robot_available) (not robot_stuck))
  )

  ;; Pause mission due to emergency
  (:action pause_mission
    :precondition (emergency_detected)
    :effect (mission_interrupted)
  )

  ;; Sync with the multi-robot team
  (:action sync_with_team
    :precondition (task_synchronized)
    :effect (multi_robot_sync)
  )

  ;; Request help from another robot when stuck
  (:action request_help_from_team
    :precondition (robot_stuck)
    :effect (multi_robot_sync)
  )

  ;; Report failure to ground control
  (:action report_failure_to_ground
    :precondition (and emergency_detected communication_delayed)
    :effect (failure_reported)
  )

  ;; Collect soil sample
  (:action collect_soil_sample
    :precondition (robot_available)
    :effect (soil_sample_collected)
  )

  ;; Collect atmospheric data
  (:action collect_atmospheric_data
    :precondition (robot_available)
    :effect (atmospheric_data_collected)
  )

  ;; Inspect infrastructure for issues
  (:action inspect_infrastructure
    :precondition (robot_available)
    :effect (infrastructure_inspected)
  )

  ;; Engage manual override from ground control
  (:action engage_ground_control_override
    :precondition (robot_available)
    :effect (ground_control_override_active)
  )

  ;; Disengage manual override
  (:action disengage_ground_control_override
    :precondition (ground_control_override_active)
    :effect (not ground_control_override_active)
  )

  ;; Detect weather hazards
  (:action detect_weather_hazard
    :precondition (robot_available)
    :effect (weather_hazard_detected)
  )

  ;; Assess damage on the robot
  (:action assess_damage
    :precondition (and robot_available robot_damaged)
    :effect (failure_reported)
  )

  ;; Attempt autonomous repair
  (:action attempt_autonomous_repair
    :precondition (robot_damaged)
    :effect (and (robot_available) (not robot_damaged))
  )

  ;; Activate safe mode in case of critical failure
  (:action safe_mode_activation
    :precondition (critical_system_failure)
    :effect (mission_interrupted)
  )

  ;; Calibrate equipment
  (:action calibrate_equipment
    :precondition (robot_available)
    :effect (equipment_calibrated)
  )

  ;; Perform routine maintenance
  (:action perform_maintenance
    :precondition (and robot_available maintenance_required)
    :effect (not maintenance_required)
  )

  ;; Store long-term data
  (:action store_long_term_data
    :precondition (findings_ready)
    :effect (long_term_data_stored)
  )

  ;; Activate redundant communication system in case of blackout
  (:action activate_redundant_communication
    :precondition (communication_blackout)
    :effect (redundant_communication_active)
  )
)
