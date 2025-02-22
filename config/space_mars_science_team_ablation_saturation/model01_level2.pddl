(define (domain space_mars_science_team)
  (:requirements :strips :typing)

  (:predicates
    ;; Sample and analysis status
    (robot_has_sample)
    (sample_detected)
    (sample_analyzed)
    (findings_ready)

    ;; Communication status
    (comm_link_active)
    (message_sent)
    (message_acknowledged)

    ;; Team coordination and safety status
    (team_synced)
    (hazard_detected)
    (system_nominal)
  )

  (:action pick_up_sample
    :precondition (and sample_detected system_nominal (not hazard_detected) (not robot_has_sample))
    :effect (and robot_has_sample
                 (not sample_detected))
  )

  (:action analyze_sample
    :precondition (and robot_has_sample system_nominal (not hazard_detected) (not sample_analyzed))
    :effect (and sample_analyzed findings_ready)
  )

  (:action report_findings
    :precondition (and findings_ready comm_link_active)
    :effect (and (not sample_analyzed)
                 (not findings_ready)
                 message_sent)
  )

  (:action receive_acknowledgment
    :precondition message_sent
    :effect (and message_acknowledged (not message_sent))
  )

  (:action drop_sample
    :precondition robot_has_sample
    :effect (and (not robot_has_sample)
                 (not sample_analyzed)
                 (not findings_ready))
  )
)
