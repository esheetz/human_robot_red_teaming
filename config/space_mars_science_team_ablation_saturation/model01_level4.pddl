(define (domain space_mars_science_team)
  (:requirements :strips :typing)
  (:types robot sample)

  (:predicates
    ;; Sample status and analysis (parameterized by sample)
    (sample_detected ?s - sample)
    (has_sample ?r - robot ?s - sample)
    (sample_analyzed ?s - sample)
    (findings_ready ?r - robot ?s - sample)

    ;; Communication status (global and per robot)
    (comm_link_active)
    (message_sent ?r - robot)
    (message_acknowledged ?r - robot)

    ;; Backup communication and mission status
    (backup_comm_active)
    (mission_active)

    ;; Team coordination and robot availability
    (team_synced)
    (free ?r - robot)

    ;; Safety conditions
    (hazard_detected)
    (system_nominal)

    ;; Environmental conditions
    (environment_normal)

    ;; Robot self-diagnosis
    (robot_healthy ?r - robot)
  )

  ;;------------------------------
  ;; Team Coordination Actions
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

  ;;------------------------------
  ;; Safety Update Actions
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

  ;;------------------------------
  ;; Self-Diagnosis and Maintenance
  (:action self_diagnosis
    :parameters (?r - robot)
    :precondition (free ?r)
    :effect robot_healthy ?r
  )

  (:action perform_maintenance
    :parameters (?r - robot)
    :precondition (not (robot_healthy ?r))
    :effect robot_healthy ?r
  )

  ;;------------------------------
  ;; Environmental Condition Updates
  (:action update_environment_normal
    :parameters ()
    :precondition (not environment_normal)
    :effect environment_normal
  )

  (:action update_environment_extreme
    :parameters ()
    :precondition environment_normal
    :effect (not environment_normal)
  )

  ;;------------------------------
  ;; Communication Blackout Protocols
  (:action activate_backup_comm
    :parameters ()
    :precondition (not comm_link_active)
    :effect backup_comm_active
  )

  (:action deactivate_backup_comm
    :parameters ()
    :precondition backup_comm_active
    :effect (not backup_comm_active)
  )

  (:action pause_mission
    :parameters ()
    :precondition (and (not comm_link_active) (not backup_comm_active) mission_active)
    :effect (not mission_active)
  )

  (:action resume_mission
    :parameters ()
    :precondition comm_link_active
    :effect mission_active
  )

  ;;------------------------------
  ;; Sample Handling Actions
  (:action pick_up_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (sample_detected ?s)
                       system_nominal
                       (not hazard_detected)
                       (free ?r)
                       team_synced
                       (robot_healthy ?r)
                       environment_normal
                       mission_active)
    :effect (and (has_sample ?r ?s)
                 (not (sample_detected ?s))
                 (not (free ?r)))
  )

  (:action analyze_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       system_nominal
                       (not hazard_detected)
                       team_synced
                       (robot_healthy ?r)
                       environment_normal
                       mission_active
                       (not (sample_analyzed ?s)))
    :effect (and (sample_analyzed ?s)
                 (findings_ready ?r ?s))
  )

  (:action report_findings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       (findings_ready ?r ?s)
                       comm_link_active
                       team_synced
                       (robot_healthy ?r)
                       environment_normal
                       mission_active)
    :effect (and (not (sample_analyzed ?s))
                 (not (findings_ready ?r ?s))
                 (message_sent ?r))
  )

  (:action receive_acknowledgment
    :parameters (?r - robot)
    :precondition (message_sent ?r)
    :effect (and (message_acknowledged ?r)
                 (not (message_sent ?r)))
  )

  (:action drop_sample
    :parameters (?r - robot ?s - sample)
    :precondition (has_sample ?r ?s)
    :effect (and (not (has_sample ?r ?s))
                 (free ?r)
                 (not (sample_analyzed ?s))
                 (not (findings_ready ?r ?s)))
  )
)
