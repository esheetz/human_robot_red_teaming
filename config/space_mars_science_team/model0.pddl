(define (domain space_mars_science_team)
  (:requirements :strips)

  (:predicates
    (robot_has_sample)    ;; Robot possesses a sample
    (sample_detected)     ;; Sample has been identified
    (sample_analyzed)     ;; Sample has been processed
    (findings_ready)      ;; Findings from analysis are available
  )

  ;; Pick up a detected sample
  (:action pick_up_sample
    :parameters ()
    :precondition (sample_detected)
    :effect (and (robot_has_sample) (not (sample_detected)))
  )

  ;; Analyze the collected sample
  (:action analyze_sample
    :parameters ()
    :precondition (robot_has_sample)
    :effect (and (sample_analyzed) (findings_ready))
  )

  ;; Report findings after analysis
  (:action report_findings
    :parameters ()
    :precondition (findings_ready)
    :effect (and (not (sample_analyzed)) (not (findings_ready)))
  )

  ;; Drop the sample after collection or analysis
  (:action drop_sample
    :parameters ()
    :precondition (robot_has_sample)
    :effect (not (robot_has_sample))
  )
)
