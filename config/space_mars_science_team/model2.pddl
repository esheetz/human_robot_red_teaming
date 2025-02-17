(define (domain space_mars_science_team)
  (:requirements :strips)

  (:predicates
    (robot_available)                   ;; Robot is available for tasks
    (robot_needs_recharge)              ;; Robot needs to recharge
    (communication_delayed)             ;; Communication delay exists
    (sample_detected)                   ;; A sample has been detected
    (sample_type_identified)            ;; The type of sample is known
    (sample_contaminated)               ;; Sample is contaminated
    (robot_has_sample)                  ;; Robot is carrying a sample
    (sample_analyzed)                   ;; Sample has been analyzed
    (findings_ready)                    ;; Findings are ready to be transmitted
    (delayed_response)                  ;; Response delayed due to communication issues
    (task_synchronized)                 ;; Robot is synchronized with others
    (environment_monitored)             ;; Environment has been monitored
    (resource_identified)               ;; Resources have been identified
    (emergency_detected)                ;; An emergency situation has been detected
    (robot_moving)                      ;; Robot is currently moving
    (robot_stuck)                       ;; Robot is stuck
    (power_low)                         ;; Robot has low power
    (mission_interrupted)               ;; Mission has been paused
    (multi_robot_sync)                  ;; Robot is in sync with multiple robots
    (failure_reported)                  ;; Failure has been reported
    (soil_sample_collected)             ;; Soil sample has been collected
    (atmospheric_data_collected)        ;; Atmospheric data has been collected
    (infrastructure_inspected)          ;; Habitat/infrastructure inspection completed
    (ground_control_override_active)    ;; Ground control override is active
  )

  ;; Scan for samples
  (:action scan_for_samples
    :parameters ()
    :precondition (robot_available)
    :effect (sample_detected)
  )

  ;; Pick up sample if detected and type is identified
  (:action pick_up_sample
    :parameters ()
    :precondition (and sample_detected sample_type_identified)
    :effect (and (robot_has_sample) (not sample_detected))
  )

  ;; Analyze sample after picking it up
  (:action analyze_sample
    :parameters ()
    :precondition (robot_has_sample)
    :effect (and (sample_analyzed) (findings_ready) (not robot_has_sample))
  )

  ;; Report findings (delayed if communication is an issue)
  (:action report_findings
    :parameters ()
    :precondition (and findings_ready communication_delayed)
    :effect (and (delayed_response) (not findings_ready))
  )

  ;; Transmit findings after delay
  (:action transmit_findings
    :parameters ()
    :precondition (and findings_ready communication_delayed)
    :effect (not delayed_response)
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
    :precondition (robot_available)
    :effect (task_synchronized)
  )

  ;; Monitor environment for resources or hazards
  (:action monitor_environment
    :parameters ()
    :precondition (robot_available)
    :effect (environment_monitored)
  )

  ;; Identify resources after monitoring the environment
  (:action identify_resources
    :parameters ()
    :precondition (and robot_available environment_monitored)
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
    :precondition (robot_available)
    :effect (robot_moving)
  )

  ;; Recharge the robot
  (:action recharge_robot
    :parameters ()
    :precondition (power_low)
    :effect (and (robot_available) (not power_low))
  )

  ;; Recover a stuck robot
  (:action self_recover
    :parameters ()
    :precondition (robot_stuck)
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
    :precondition (task_synchronized)
    :effect (multi_robot_sync)
  )

  ;; Request help from another robot when stuck
  (:action request_help_from_team
    :parameters ()
    :precondition (robot_stuck)
    :effect (multi_robot_sync)
  )

  ;; Report failure to ground control
  (:action report_failure_to_ground
    :parameters ()
    :precondition (and emergency_detected communication_delayed)
    :effect (failure_reported)
  )

  ;; Collect soil sample
  (:action collect_soil_sample
    :parameters ()
    :precondition (robot_available)
    :effect (soil_sample_collected)
  )

  ;; Collect atmospheric data
  (:action collect_atmospheric_data
    :parameters ()
    :precondition (robot_available)
    :effect (atmospheric_data_collected)
  )

  ;; Inspect infrastructure for issues
  (:action inspect_infrastructure
    :parameters ()
    :precondition (robot_available)
    :effect (infrastructure_inspected)
  )

  ;; Engage manual override from ground control
  (:action engage_ground_control_override
    :parameters ()
    :precondition (robot_available)
    :effect (ground_control_override_active)
  )

  ;; Disengage manual override
  (:action disengage_ground_control_override
    :parameters ()
    :precondition (ground_control_override_active)
    :effect (not ground_control_override_active)
  )
)
