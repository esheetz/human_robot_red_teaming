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

    ;; Backup communication and mission status (multiple backups)
    (backup_comm_active)
    (backup_comm_active2)
    (backup_comm_active3)
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

    ;; Logging and verification predicates
    (diagnostic_log_sent ?r - robot)
    (environment_log_sent)
    (team_status_logged)
    (hardware_status_verified ?r - robot)
    (environment_verified)

    ;; Composite operational state: true when all conditions for safe operation hold
    (operational_state ?r - robot)

    ;; --- New Contingency Predicates ---
    ;; Indicates that the robot has entered safe mode due to degraded communication.
    (safe_mode ?r - robot)

    ;; Indicates that a diagnostic log is buffered (not yet sent).
    (buffered_diagnostic_log ?r - robot)

    ;; Indicates that an environment log is buffered (not yet sent).
    (buffered_environment_log)
  )

  ;;--------------------------------------------------
  ;; Composite Operational State Update and Invalidation (as before)
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

  (:action invalidate_operational_state
    :parameters (?r - robot)
    :precondition (operational_state ?r)
    :effect (not (operational_state ?r))
  )

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

  (:action invalidate_operational_state_due_to_communication
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not communication_ok))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_health
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (robot_healthy ?r)))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_calibration
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (calibrated ?r)))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_resources
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (sufficient_resources ?r)))
    :effect (not (operational_state ?r))
  )

  ;;--------------------------------------------------
  ;; Communication Update: if all channels are down then communication_ok becomes false.
  (:action update_communication_not_ok
    :parameters ()
    :precondition (and (not comm_link_active)
                       (not backup_comm_active)
                       (not backup_comm_active2)
                       (not backup_comm_active3)
                       communication_ok)
    :effect (not communication_ok)
  )

  ;;--------------------------------------------------
  ;; Team Coordination Actions and Proactive Status Logging
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

  (:action send_team_status
    :parameters ()
    :precondition (and comm_link_active team_synced)
    :effect team_status_logged
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
  ;; Communication Blackout and Backup Protocols
  (:action activate_backup_comm
    :parameters ()
    :precondition (not comm_link_active)
    :effect (and backup_comm_active communication_ok)
  )

  (:action activate_backup_comm2
    :parameters ()
    :precondition (and (not comm_link_active) (not backup_comm_active2))
    :effect backup_comm_active2
  )

  (:action activate_backup_comm3
    :parameters ()
    :precondition (and (not comm_link_active) (not backup_comm_active3))
    :effect backup_comm_active3
  )

  (:action deactivate_backup_comm
    :parameters ()
    :precondition backup_comm_active
    :effect (not backup_comm_active)
  )

  (:action pause_mission
    :parameters ()
    :precondition (and (not comm_link_active)
                       (not backup_comm_active)
                       (not backup_comm_active2)
                       (not backup_comm_active3)
                       mission_active)
    :effect (and (not mission_active) (not communication_ok))
  )

  (:action resume_mission
    :parameters ()
    :precondition comm_link_active
    :effect (and mission_active communication_ok)
  )

  (:action switch_to_primary_comm
    :parameters ()
    :precondition (and comm_link_active (or backup_comm_active backup_comm_active2 backup_comm_active3))
    :effect (and (not backup_comm_active)
                 (not backup_comm_active2)
                 (not backup_comm_active3))
  )

  ;;--------------------------------------------------
  ;; Logging and Ground Control Verification Actions
  (:action verify_sample_readings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s) comm_link_active team_synced)
    :effect (sample_verified ?r ?s)
  )
  
  (:action send_diagnostic_log
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) comm_link_active team_synced (not (safe_mode ?r)))
    :effect (diagnostic_log_sent ?r)
  )

  (:action send_environment_log
    :parameters ()
    :precondition (and comm_link_active team_synced (not (exists (?r - robot) (safe_mode ?r))))
    :effect environment_log_sent
  )

  (:action verify_hardware_status
    :parameters (?r - robot)
    :precondition (and (diagnostic_log_sent ?r) comm_link_active team_synced)
    :effect (hardware_status_verified ?r)
  )

  (:action verify_environment_readings
    :parameters ()
    :precondition (and environment_log_sent comm_link_active team_synced)
    :effect environment_verified
  )

  ;; Invalidation for logging/verification mismatches:
  (:action invalidate_hardware_status_verification
    :parameters (?r - robot)
    :precondition (and (hardware_status_verified ?r) (not (diagnostic_log_sent ?r)))
    :effect (not (hardware_status_verified ?r))
  )

  (:action invalidate_environment_verification
    :parameters ()
    :precondition (and environment_verified (not environment_log_sent))
    :effect (not environment_verified)
  )

  ;; --- New Contingency Actions for Communication and Logging ---

  ;; Enter safe mode if communication is lost.
  (:action enter_safe_mode
    :parameters (?r - robot)
    :precondition (not communication_ok)
    :effect (safe_mode ?r)
  )

  ;; Exit safe mode when communication is restored.
  (:action exit_safe_mode
    :parameters (?r - robot)
    :precondition communication_ok
    :effect (not (safe_mode ?r))
  )

  ;; Buffer diagnostic log if it could not be sent.
  (:action buffer_diagnostic_log
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not diagnostic_log_sent ?r))
    :effect (buffered_diagnostic_log ?r)
  )

  ;; Retry sending a buffered diagnostic log when communication is available.
  (:action retry_send_diagnostic_log
    :parameters (?r - robot)
    :precondition (and buffered_diagnostic_log ?r comm_link_active team_synced)
    :effect (and diagnostic_log_sent ?r (not buffered_diagnostic_log ?r))
  )

  ;; Buffer environment log if it could not be sent.
  (:action buffer_environment_log
    :parameters ()
    :precondition (and environment_verified (not environment_log_sent))
    :effect buffered_environment_log
  )

  ;; Retry sending a buffered environment log when communication is available.
  (:action retry_send_environment_log
    :parameters ()
    :precondition (and buffered_environment_log comm_link_active team_synced)
    :effect (and environment_log_sent (not buffered_environment_log))
  )

  ;;--------------------------------------------------
  ;; Sample Handling Actions (using composite operational_state)
  (:action pick_up_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (sample_detected ?s)
                       free ?r
                       team_synced
                       (operational_state ?r)
                       (not (safe_mode ?r)))
    :effect (and (has_sample ?r ?s)
                 (not (sample_detected ?s))
                 (not (free ?r)))
  )

  (:action analyze_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       team_synced
                       (operational_state ?r)
                       (not (sample_analyzed ?s))
                       (not (safe_mode ?r)))
    :effect (and (sample_analyzed ?s)
                 (findings_ready ?r ?s))
  )

  (:action report_findings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       (findings_ready ?r ?s)
                       comm_link_active   ; reporting requires primary channel.
                       team_synced
                       (operational_state ?r)
                       (not (safe_mode ?r)))
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

  (:action emergency_abort
    :parameters (?r - robot ?s - sample)
    :precondition (has_sample ?r ?s)
    :effect (and (not (has_sample ?r ?s))
                 (free ?r)
                 (not (sample_analyzed ?s))
                 (not (findings_ready ?r ?s)))
  )
)
