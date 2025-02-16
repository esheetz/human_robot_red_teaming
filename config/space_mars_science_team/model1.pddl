(define (domain space_mars_science_team)
  (:requirements :strips)

  (:predicates
    (robot_available)              ;; Robot is available for tasks
    (robot_needs_recharge)         ;; Robot requires recharging
    (communication_delayed)        ;; Communication with base is delayed
    (sample_detected)              ;; A sample has been identified
    (sample_type_identified)       ;; Sample type has been classified
    (sample_contaminated)          ;; Sample is contaminated
    (robot_has_sample)             ;; Robot is carrying a sample
    (sample_analyzed)              ;; Sample has been analyzed
    (findings_ready)               ;; Findings are available for transmission
    (delayed_response)             ;; Response was delayed due to communication issues
    (task_synchronized)            ;; Robot has synchronized with other robots
    (environment_monitored)        ;; Robot has monitored the surroundings
    (resource_identified)          ;; Resources have been identified
    (emergency_detected)           ;; Emergency situation has been detected
  )

  ;; Scan for samples
  (:action scan_for_samples
    :precondition (robot_available)
    :effect (sample_detected)
  )

  ;; Pick up sample if detected and identified
  (:action pick_up_sample
    :precondition (and sample_detected sample_type_identified)
    :effect (and (robot_has_sample) (not sample_detected))
  )

  ;; Analyze collected sample
  (:action analyze_sample
    :precondition (robot_has_sample)
    :effect (and (sample_analyzed) (findings_ready) (not robot_has_sample))
  )

  ;; Report findings (delayed if communication is an issue)
  (:action report_findings
    :precondition (and findings_ready communication_delayed)
    :effect (and (delayed_response) (not findings_ready))
  )

  ;; Transmit findings after a delay
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
)
