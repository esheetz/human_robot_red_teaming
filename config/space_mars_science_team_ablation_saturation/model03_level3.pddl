(define (domain space_mars_science_team)
  (:requirements :strips :typing)
  (:types robot sample)

  (:predicates
    ;; Sample status and analysis (parameterized by sample)
    (sample_detected ?s - sample)
    (has_sample ?r - robot ?s - sample)
    (sample_analyzed ?s - sample)
    (findings_ready ?r - robot ?s - sample)
    (sample_verified ?r - robot ?s - sample)

    ;; Communication status (global and per robot)
    (comm_link_active)
    (message_sent ?r - robot)
    (message_acknowledged ?r - robot)

    ;; Backup communication and mission status
    (backup_comm_active)
    (mission_active)

    ;; Communication availability: true if at least one channel is active
    (communication_ok)

    ;; Team coordination and robot availability
    (team_synced)
    (free ?r - robot)

    ;; Safety conditions
    (hazard_detected)
    (system_nominal)

    ;; Detailed environmental conditions
    (temperature_normal)
    (radiation_normal)
    (dust_normal)

    ;; Robot self-diagnosis and calibration
    (robot_healthy ?r - robot)
    (calibrated ?r - robot)

    ;; Resource status (e.g., battery level, power)
    (sufficient_resources ?r - robot)

    ;; Ground control verification for environment readings
    (environment_verified)

    ;; Composite operational state: true when all conditions for safe operation hold
    (operational_state ?r - robot)
  )

  ;;--------------------------------------------------
  ;; Composite Operational State Update
  (:action update_operational_state
    :parameters (?r - robot)
    :precondition (and (robot_healthy ?r)
                        system_nominal
                        mission_active
                        communication_ok
                        (calibrated ?r)
                        (sufficient_resources ?r)
                        temperature_normal
                        radiation_normal
                        dust_normal)
    :effect (operational_state ?r)
  )

  ;; A general invalidation action (can be used if any condition fails)
  (:action invalidate_operational_state
    :parameters (?r - robot)
    :precondition (operational_state ?r)
    :effect (not (operational_state ?r))
  )

  ;; Invalidate operational state due to environmental changes
  (:action invalidate_operational_state_due_to_temperature
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not temperature_normal))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_radiation
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not radiation_normal))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_dust
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not dust_normal))
    :effect (not (operational_state ?r))
  )

  ;; Invalidate operational state due to communication issues
  (:action invalidate_operational_state_due_to_communication
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not communication_ok))
    :effect (not (operational_state ?r))
  )

  ;; Invalidate operational state due to robot health failure
  (:action invalidate_operational_state_due_to_health
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (robot_healthy ?r)))
    :effect (not (operational_state ?r))
  )

  ;; Invalidate operational state due to lack of calibration
  (:action invalidate_operational_state_due_to_calibration
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (calibrated ?r)))
    :effect (not (operational_state ?r))
  )

  ;; Invalidate operational state due to insufficient resources
  (:action invalidate_operational_state_due_to_resources
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (sufficient_resources ?r)))
    :effect (not (operational_state ?r))
  )

  ;;--------------------------------------------------
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

  ;;--------------------------------------------------
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

  ;;--------------------------------------------------
  ;; Self-Diagnosis, Calibration, and Resource Checks
  (:action self_diagnose
    :parameters (?r - robot)
    :precondition (free ?r)
    :effect (robot_healthy ?r)
  )

  (:action self_calibrate
    :parameters (?r - robot)
    :precondition (free ?r)
    :effect (calibrated ?r)
  )

  (:action check_resources
    :parameters (?r - robot)
    :precondition (free ?r)
    :effect (sufficient_resources ?r)
  )

  (:action perform_maintenance
    :parameters (?r - robot)
    :precondition (and (free ?r) (not (robot_healthy ?r)))
    :effect (robot_healthy ?r)
  )

  ;;--------------------------------------------------
  ;; Detailed Environmental Condition Updates
  (:action update_temperature_normal
    :parameters ()
    :precondition (not temperature_normal)
    :effect temperature_normal
  )

  (:action update_temperature_extreme
    :parameters ()
    :precondition temperature_normal
    :effect (not temperature_normal)
  )

  (:action update_radiation_normal
    :parameters ()
    :precondition (not radiation_normal)
    :effect radiation_normal
  )

  (:action update_radiation_extreme
    :parameters ()
    :precondition radiation_normal
    :effect (not radiation_normal)
  )

  (:action update_dust_normal
    :parameters ()
    :precondition (not dust_normal)
    :effect dust_normal
  )

  (:action update_dust_extreme
    :parameters ()
    :precondition dust_normal
    :effect (not dust_normal)
  )

  (:action pause_mission_due_to_environment
    :parameters ()
    :precondition (and (or (not temperature_normal) (not radiation_normal) (not dust_normal)) mission_active)
    :effect (not mission_active)
  )

  ;;--------------------------------------------------
  ;; Communication Blackout Protocols
  (:action activate_backup_comm
    :parameters ()
    :precondition (not comm_link_active)
    :effect (and backup_comm_active communication_ok)
  )

  (:action deactivate_backup_comm
    :parameters ()
    :precondition backup_comm_active
    :effect (not backup_comm_active)
  )

  (:action pause_mission
    :parameters ()
    :precondition (and (not comm_link_active) (not backup_comm_active) mission_active)
    :effect (and (not mission_active) (not communication_ok))
  )

  (:action resume_mission
    :parameters ()
    :precondition comm_link_active
    :effect (and mission_active communication_ok)
  )

  (:action update_communication_not_ok
    :parameters ()
    :precondition (and (not comm_link_active) (not backup_comm_active) communication_ok)
    :effect (not communication_ok)
  )

  ;; New: Switch to primary communication if it becomes available while backup is active.
  (:action switch_to_primary_comm
    :parameters ()
    :precondition (and comm_link_active backup_comm_active)
    :effect (not backup_comm_active)
  )

  ;;--------------------------------------------------
  ;; Ground Control Verification Actions
  (:action verify_sample_readings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s) comm_link_active team_synced)
    :effect (sample_verified ?r ?s)
  )

  (:action verify_environment_readings
    :parameters (?r - robot)
    :precondition (and comm_link_active team_synced)
    :effect environment_verified
  )

  ;;--------------------------------------------------
  ;; Sample Handling Actions (using composite operational_state)
  (:action pick_up_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (sample_detected ?s)
                       free ?r
                       team_synced
                       (operational_state ?r))
    :effect (and (has_sample ?r ?s)
                 (not (sample_detected ?s))
                 (not (free ?r)))
  )

  (:action analyze_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       team_synced
                       (operational_state ?r)
                       (not (sample_analyzed ?s)))
    :effect (and (sample_analyzed ?s)
                 (findings_ready ?r ?s))
  )

  (:action report_findings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       (findings_ready ?r ?s)
                       comm_link_active   ; reporting requires primary channel.
                       team_synced
                       (operational_state ?r))
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

  ;; Revised: Emergency abort action allows a busy robot to interrupt its current task.
  (:action emergency_abort
    :parameters (?r - robot ?s - sample)
    :precondition (has_sample ?r ?s)
    :effect (and (not (has_sample ?r ?s))
                 (free ?r)
                 (not (sample_analyzed ?s))
                 (not (findings_ready ?r ?s)))
  )
)
