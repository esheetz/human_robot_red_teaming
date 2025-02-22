(define (domain space_mars_science_team)
  (:requirements :strips :typing)
  (:types robot sample)

  (:predicates
    ;; Sample status and analysis (parameterized by sample)
    (sample_detected ?s - sample)
    (has_sample ?r - robot ?s - sample)
    (sample_analyzed ?s - sample)
    (findings_ready ?r - robot ?s - sample)

    ;; Communication status (per robot)
    (comm_link_active)
    (message_sent ?r - robot)
    (message_acknowledged ?r - robot)

    ;; Team coordination and robot availability
    (team_synced)
    (free ?r - robot)

    ;; Safety conditions
    (hazard_detected)
    (system_nominal)
  )

  ;; Team coordination actions
  (:action sync_team
    :parameters ()
    :precondition comm_link_active
    :effect team_synced
  )

  (:action unsync_team
    :parameters ()
    :precondition team_synced
    :effect (not team_synced)
  )

  ;; Safety update actions
  (:action detect_hazard
    :parameters ()
    :precondition system_nominal
    :effect (and hazard_detected (not system_nominal))
  )

  (:action clear_hazard
    :parameters ()
    :precondition hazard_detected
    :effect (and system_nominal (not hazard_detected))
  )

  ;; Pick up a sample (only if the robot is free, team is synced, and safety is ensured)
  (:action pick_up_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (sample_detected ?s)
                       system_nominal
                       (not hazard_detected)
                       (free ?r)
                       team_synced)
    :effect (and (has_sample ?r ?s)
                 (not (sample_detected ?s))
                 (not (free ?r)))
  )

  ;; Analyze the sample (the robot must be holding the sample and conditions must be safe)
  (:action analyze_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       system_nominal
                       (not hazard_detected)
                       team_synced
                       (not (sample_analyzed ?s)))
    :effect (and (sample_analyzed ?s)
                 (findings_ready ?r ?s))
  )

  ;; Report findings (requires an active comm link and team coordination)
  (:action report_findings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       (findings_ready ?r ?s)
                       comm_link_active
                       team_synced)
    :effect (and (not (sample_analyzed ?s))
                 (not (findings_ready ?r ?s))
                 (message_sent ?r))
  )

  ;; Receive acknowledgment for the reported findings
  (:action receive_acknowledgment
    :parameters (?r - robot)
    :precondition (message_sent ?r)
    :effect (and (message_acknowledged ?r)
                 (not (message_sent ?r)))
  )

  ;; Drop the sample (resetting analysis and freeing the robot)
  (:action drop_sample
    :parameters (?r - robot ?s - sample)
    :precondition (has_sample ?r ?s)
    :effect (and (not (has_sample ?r ?s))
                 (free ?r)
                 (not (sample_analyzed ?s))
                 (not (findings_ready ?r ?s)))
  )
)
