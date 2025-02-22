(define (domain space_mars_science_team)
  (:requirements :strips)

  (:predicates
    (robot_has_sample)
    (sample_detected)
    (sample_analyzed)
    (findings_ready)
  )

  (:action pick_up_sample
    :precondition (sample_detected)
    :effect (and (robot_has_sample) (not (sample_detected)))
  )

  (:action analyze_sample
    :precondition (robot_has_sample)
    :effect (and (sample_analyzed) (findings_ready))
  )

  (:action report_findings
    :precondition (findings_ready)
    :effect (and (not (sample_analyzed)) (not (findings_ready)))
  )

  (:action drop_sample
    :precondition (robot_has_sample)
    :effect (not (robot_has_sample))
  )
)
