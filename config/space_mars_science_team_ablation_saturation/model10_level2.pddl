(define (domain space_mars_science_team)
  (:requirements :strips :typing :quantified-preconditions)
  (:types robot sample level)

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

    ;; Detailed environmental conditions with granular sensor levels
    (temperature_normal)
    (radiation_normal)
    (dust_normal)
    (temperature_level ?l - level)
    (radiation_level ?l - level)
    (dust_level ?l - level)

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

    ;; Ground control periodic status update
    (gc_status_updated)

    ;; Composite operational state: true when all conditions for safe operation hold
    (operational_state ?r - robot)

    ;; --- Contingency Predicates ---
    (safe_mode ?r - robot)
    (buffered_diagnostic_log ?r - robot)
    (buffered_environment_log)

    ;; --- New: Grace Period Predicate ---
    (grace_period_active ?r - robot)

    ;; --- New: Extended Grace Period Predicate (for persistent issues) ---
    (extended_grace_period ?r - robot)

    ;; --- New: Adaptive Sensor Threshold Adjustment ---
    (adaptive_sensor_adjusted ?r - robot)

    ;; --- New: Backup Data Sharing ---
    (backup_data_shared ?r - robot)

    ;; --- New: Failure Notification Predicate ---
    (failure_notified ?r - robot)

    ;; --- New: Timing and Buffering Predicates ---
    (buffer_timer_expired ?r - robot)
    (state_update_timer_expired ?r - robot)
    (waiting ?r - robot)

    ;; --- Distributed Reasoning Predicates ---
    (global_operational_consensus)
    (elected_leader ?r - robot)
    (leader_exists)
    (consensus_confirmed)

    ;; --- Inter-Agent Status Sharing ---
    (status_shared ?r - robot)
    ;; Severe failure mode predicates
    (communication_blackout ?r - robot)
    (environmental_alert_active ?r - robot)
    (battery_critical ?r - robot)
    (fallback_leader_election_in_progress)
    (hardware_fault_detected ?r - robot)

    ;; Predictive maintenance and redundancy
    (predictive_fault_detected ?r - robot)
    (redundancy_system_active ?r - robot)

    ;; Invalid state prevention predicates
    (team_synced)
    (leader_exists)
    (sheltered ?r - robot)
    (backup_system_active ?r - robot)
    (operational_state ?r - robot)
    (environmental_scan_complete ?r - robot)

    ;; Energy distribution predicates
    (energy_sufficient ?r - robot)
    (energy_transferred ?r1 - robot ?r2 - robot)
    (minimum_energy_threshold ?r - robot)

    ;; Hardware recovery predicates
    (diagnostics_complete ?r - robot)
    (full_system_check_complete ?r - robot)

    ;; New failure and recovery attempt predicates
    (failed_recovery_attempt ?r - robot)
  )

  ;; --- Leader Election and Global Consensus ---
  (:action elect_leader
    :parameters (?r - robot)
    :precondition (and (free ?r) team_synced (not leader_exists))
    :effect (and (elected_leader ?r) (leader_exists))
  )

  (:action collect_status_updates
    :parameters (?leader - robot)
    :precondition (and (elected_leader ?leader) team_synced)
    :effect global_operational_consensus
  )

  ;; New: Confirm global consensus if every robot has shared its status.
  (:action confirm_global_consensus
    :parameters ()
    :precondition (forall (?r - robot) (status_shared ?r))
    :effect consensus_confirmed
  )

  (:action invalidate_global_consensus
    :parameters ()
    :precondition (not (forall (?r - robot) (status_shared ?r)))
    :effect (not consensus_confirmed)
  )

  ;; --- Granular Sensor Level Update Actions ---
  (:action set_temperature_level
    :parameters (?l - level)
    :precondition temperature_normal
    :effect (temperature_level ?l)
  )

  (:action set_radiation_level
    :parameters (?l - level)
    :precondition radiation_normal
    :effect (radiation_level ?l)
  )

  (:action set_dust_level
    :parameters (?l - level)
    :precondition dust_normal
    :effect (dust_level ?l)
  )

  ;; --- Adaptive Sensor Threshold Actions ---
  (:action adjust_sensor_threshold
    :parameters (?r - robot)
    :precondition (and (free ?r) (not temperature_normal))
    :effect (adaptive_sensor_adjusted ?r)
  )

  (:action reset_sensor_threshold_adjustment
    :parameters (?r - robot)
    :precondition temperature_normal
    :effect (not (adaptive_sensor_adjusted ?r))
  )

  (:action auto_reset_adaptive_sensor
    :parameters (?r - robot)
    :precondition (and temperature_normal (adaptive_sensor_adjusted ?r))
    :effect (not (adaptive_sensor_adjusted ?r))
  )

  ;; --- Composite Operational State Update Actions ---
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
                       dust_normal
                       gc_status_updated
                       global_operational_consensus
                       consensus_confirmed
                       (not (safe_mode ?r))
                       (not (grace_period_active ?r))
                       (not (extended_grace_period ?r)))
    :effect (operational_state ?r)
  )

  (:action update_operational_state_adaptive
    :parameters (?r - robot)
    :precondition (and (robot_healthy ?r)
                       system_nominal
                       mission_active
                       communication_ok
                       (calibrated ?r)
                       (sufficient_resources ?r)
                       gc_status_updated
                       global_operational_consensus
                       consensus_confirmed
                       (adaptive_sensor_adjusted ?r)
                       (not (safe_mode ?r))
                       (not (grace_period_active ?r))
                       (not (extended_grace_period ?r)))
    :effect (operational_state ?r)
  )

  (:action invalidate_operational_state
    :parameters (?r - robot)
    :precondition (operational_state ?r)
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_temperature
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not temperature_normal)
                       (not (grace_period_active ?r)))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_radiation
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not radiation_normal)
                       (not (grace_period_active ?r)))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_dust
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not dust_normal)
                       (not (grace_period_active ?r)))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_communication
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not communication_ok)
                       (not (grace_period_active ?r)))
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

  (:action invalidate_operational_state_due_to_gc_status
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not gc_status_updated))
    :effect (not (operational_state ?r))
  )

  (:action invalidate_operational_state_due_to_safe_mode
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (safe_mode ?r))
    :effect (not (operational_state ?r))
  )

  ;; --- Grace Period Management Actions ---
  (:action enter_grace_period
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (grace_period_active ?r)))
    :effect (grace_period_active ?r)
  )

  (:action exit_grace_period
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) communication_ok)
    :effect (and (not (grace_period_active ?r)) (not (extended_grace_period ?r)))
  )

  (:action end_grace_period_due_to_comm_failure
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not communication_ok))
    :effect (and (not (grace_period_active ?r)) (not (operational_state ?r)) (not (extended_grace_period ?r)))
  )

  (:action end_grace_period_due_to_sensor_failure
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not temperature_normal))
    :effect (and (not (grace_period_active ?r)) (not (operational_state ?r)) (not (extended_grace_period ?r)))
  )

  ;; --- Extended Grace Period (Adaptive Threshold Contingency) ---
  (:action extend_grace_period
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not (extended_grace_period ?r)))
    :effect (extended_grace_period ?r)
  )

  (:action exit_extended_grace_period
    :parameters (?r - robot)
    :precondition (and (extended_grace_period ?r) communication_ok)
    :effect (not (extended_grace_period ?r))
  )

  ;; --- New: Timer Clearing Actions ---
  (:action clear_buffer_timer
    :parameters (?r - robot)
    :precondition (buffer_timer_expired ?r)
    :effect (not (buffer_timer_expired ?r))
  )

  (:action clear_state_update_timer
    :parameters (?r - robot)
    :precondition (state_update_timer_expired ?r)
    :effect (not (state_update_timer_expired ?r))
  )

  ;; --- New: Force Retry/Update Actions ---
  (:action force_retry_send_diagnostic_log
    :parameters (?r - robot)
    :precondition (and buffered_diagnostic_log ?r, buffer_timer_expired ?r, comm_link_active, team_synced)
    :effect (and diagnostic_log_sent ?r (not buffered_diagnostic_log ?r) (not buffer_timer_expired ?r))
  )

  (:action force_state_update
    :parameters (?r - robot)
    :precondition (state_update_timer_expired ?r)
    :effect (update_operational_state ?r)
  )

  (:action update_communication_not_ok
    :parameters ()
    :precondition (and (not comm_link_active)
                       (not backup_comm_active)
                       (not backup_comm_active2)
                       (not backup_comm_active3)
                       communication_ok)
    :effect (not communication_ok)
  )

  ;; --- Distributed Reasoning: Team Coordination and Consensus ---
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

  (:action share_status
    :parameters (?r - robot)
    :precondition (and comm_link_active team_synced)
    :effect (status_shared ?r)
  )

  (:action send_team_status
    :parameters ()
    :precondition (and comm_link_active team_synced)
    :effect team_status_logged
  )

  ;; --- Ground Control Status Update ---
  (:action ground_control_status_update
    :parameters ()
    :precondition comm_link_active
    :effect gc_status_updated
  )

  ;; --- Safety Update Actions ---
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

  ;; --- Self-Diagnosis, Calibration, and Resource Checks ---
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

  ;; --- Detailed Environmental Condition Updates ---
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

  ;; --- Communication Blackout and Backup Protocols ---
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

  ;; --- Logging and Ground Control Verification Actions ---
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

  ;; --- Contingency Actions for Communication and Logging ---
  (:action enter_safe_mode
    :parameters (?r - robot)
    :precondition (not communication_ok)
    :effect (and (safe_mode ?r) (not (operational_state ?r)))
  )

  (:action exit_safe_mode
    :parameters (?r - robot)
    :precondition communication_ok
    :effect (not (safe_mode ?r))
  )

  (:action buffer_diagnostic_log
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not diagnostic_log_sent ?r))
    :effect (buffered_diagnostic_log ?r)
  )

  (:action retry_send_diagnostic_log
    :parameters (?r - robot)
    :precondition (and buffered_diagnostic_log ?r comm_link_active team_synced)
    :effect (and diagnostic_log_sent ?r (not buffered_diagnostic_log ?r))
  )

  (:action buffer_environment_log
    :parameters ()
    :precondition (and environment_verified (not environment_log_sent))
    :effect buffered_environment_log
  )

  (:action retry_send_environment_log
    :parameters ()
    :precondition (and buffered_environment_log comm_link_active team_synced)
    :effect (and environment_log_sent (not buffered_environment_log))
  )

  (:action retract_status_shared_due_to_safe_mode
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (status_shared ?r))
    :effect (not (status_shared ?r))
  )

  (:action invalidate_hardware_status_due_to_safe_mode
    :parameters (?r - robot)
    :precondition (and (hardware_status_verified ?r) (safe_mode ?r))
    :effect (not (hardware_status_verified ?r))
  )

  (:action invalidate_environment_verification_due_to_comm_loss
    :parameters ()
    :precondition (and environment_verified (not communication_ok))
    :effect (not environment_verified)
  )

  ;; --- Grace Period Management Actions ---
  (:action enter_grace_period
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (grace_period_active ?r)))
    :effect (grace_period_active ?r)
  )

  (:action exit_grace_period
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) communication_ok)
    :effect (and (not (grace_period_active ?r)) (not (extended_grace_period ?r)))
  )

  (:action end_grace_period_due_to_comm_failure
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not communication_ok))
    :effect (and (not (grace_period_active ?r)) (not (operational_state ?r)) (not (extended_grace_period ?r)))
  )

  (:action end_grace_period_due_to_sensor_failure
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not temperature_normal))
    :effect (and (not (grace_period_active ?r)) (not (operational_state ?r)) (not (extended_grace_period ?r)))
  )

  ;; --- Extended Grace Period (Adaptive Threshold Contingency) ---
  (:action extend_grace_period
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not (extended_grace_period ?r)))
    :effect (extended_grace_period ?r)
  )

  (:action exit_extended_grace_period
    :parameters (?r - robot)
    :precondition (and (extended_grace_period ?r) communication_ok)
    :effect (not (extended_grace_period ?r))
  )

  ;; --- Risk Mitigation Actions for Severe Failures ---
  (:action notify_ground_control_of_failure
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (not (operational_state ?r)) (not (failure_notified ?r)) comm_link_active)
    :effect (failure_notified ?r)
  )

  (:action system_reset
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (failure_notified ?r) comm_link_active)
    :effect (and (not safe_mode ?r)
                 (not grace_period_active ?r)
                 (not extended_grace_period ?r)
                 (free ?r)
                 (robot_healthy ?r)
                 (calibrated ?r)
                 (sufficient_resources ?r)
                 (not failure_notified ?r))
  )

  (:action emergency_reboot
    :parameters (?r - robot)
    :precondition (and (extended_grace_period ?r) (not (safe_mode ?r)) (not (operational_state ?r)))
    :effect (and (robot_healthy ?r) (calibrated ?r) (sufficient_resources ?r)
                 (free ?r) (not (extended_grace_period ?r)))
  )

  (:action emergency_shutdown
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (not communication_ok))
    :effect (and (not (robot_healthy ?r)) (not (operational_state ?r)))
  )

  (:action attempt_reboot_comm
    :parameters ()
    :precondition (not communication_ok)
    :effect comm_link_active
  )

  (:action switch_to_backup_sensor_temperature
    :parameters (?r - robot ?l - level)
    :precondition (not temperature_normal)
    :effect temperature_normal
  )

  (:action switch_to_backup_sensor_radiation
    :parameters (?r - robot ?l - level)
    :precondition (not radiation_normal)
    :effect radiation_normal
  )

  (:action switch_to_backup_sensor_dust
    :parameters (?r - robot ?l - level)
    :precondition (not dust_normal)
    :effect dust_normal
  )

  ;; --- Periodic Backup Data Broadcasting ---
  (:action broadcast_backup_data
    :parameters (?r - robot)
    :precondition (and comm_link_active team_synced
                       (or buffered_diagnostic_log ?r buffered_environment_log)
                       (not (safe_mode ?r)))
    :effect (backup_data_shared ?r)
  )

  ;; --- Sample Handling Actions ---
  (:action pick_up_sample
    :parameters (?r - robot ?s - sample)
    :precondition (and (sample_detected ?s)
                       free ?r
                       team_synced
                       (operational_state ?r)
                       (not (safe_mode ?r))
                       (not (grace_period_active ?r)))
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
                       (not (safe_mode ?r))
                       (not (grace_period_active ?r)))
    :effect (and (sample_analyzed ?s)
                 (findings_ready ?r ?s))
  )

  (:action report_findings
    :parameters (?r - robot ?s - sample)
    :precondition (and (has_sample ?r ?s)
                       (findings_ready ?r ?s)
                       comm_link_active
                       team_synced
                       (operational_state ?r)
                       (not (safe_mode ?r))
                       (not (grace_period_active ?r)))
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

  ;; --- Fault Correction Actions ---
  (:action human_repair_robot
    :parameters (?r - robot)
    :precondition (and (not (robot_healthy ?r)) comm_link_active)
    :effect (robot_healthy ?r)
  )

  (:action robot_repair_robot
    :parameters (?r1 - robot ?r2 - robot)
    :precondition (and (operational_state ?r1) team_synced (not (robot_healthy ?r2)))
    :effect (robot_healthy ?r2)
  )

  ;; --- New: Timing and Buffering Actions ---
  (:action start_buffer_timer
    :parameters (?r - robot)
    :precondition (buffered_diagnostic_log ?r)
    :effect (buffer_timer_expired ?r)
  )

  (:action start_state_update_timer
    :parameters (?r - robot)
    :precondition (operational_state ?r)
    :effect (state_update_timer_expired ?r)
  )

  (:action force_retry_send_diagnostic_log
    :parameters (?r - robot)
    :precondition (and buffered_diagnostic_log ?r, buffer_timer_expired ?r, comm_link_active, team_synced)
    :effect (and diagnostic_log_sent ?r (not buffered_diagnostic_log ?r) (not buffer_timer_expired ?r))
  )

  (:action force_state_update
    :parameters (?r - robot)
    :precondition (state_update_timer_expired ?r)
    :effect (update_operational_state ?r)
  )

  ;; --- Distributed Reasoning: Team Coordination and Consensus ---
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

  (:action share_status
    :parameters (?r - robot)
    :precondition (and comm_link_active team_synced)
    :effect (status_shared ?r)
  )

  (:action send_team_status
    :parameters ()
    :precondition (and comm_link_active team_synced)
    :effect team_status_logged
  )

  ;; --- Ground Control Status Update ---
  (:action ground_control_status_update
    :parameters ()
    :precondition comm_link_active
    :effect gc_status_updated
  )

  ;; --- Safety Update Actions ---
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

  ;; --- Self-Diagnosis, Calibration, and Resource Checks ---
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

  ;; --- Detailed Environmental Condition Updates ---
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

  ;; Constraints to prevent invalid state combinations

  ;; Redundancy system activation must follow predictive fault detection
  (:action prevent_redundancy_without_fault
    :parameters (?r - robot)
    :precondition (not (predictive_fault_detected ?r))
    :effect (not (redundancy_system_active ?r))
  )

  ;; Prevent team synchronization during communication blackout
  (:action invalidate_team_sync_on_blackout
    :parameters (?r - robot)
    :precondition (communication_blackout ?r)
    :effect (not team_synced)
  )

  ;; Predictive fault detection
  (:action run_predictive_fault_detection
    :parameters (?r - robot)
    :precondition (and (operational_state ?r) (not (predictive_fault_detected ?r)))
    :effect (predictive_fault_detected ?r)
  )

  ;; Activate redundancy system
  (:action activate_redundancy_system
    :parameters (?r - robot)
    :precondition (predictive_fault_detected ?r)
    :effect (redundancy_system_active ?r)
  )

  ;; Re-sync after reconnection
  (:action resync_team_after_reconnect
    :parameters (?r - robot)
    :precondition (and (not (communication_blackout ?r)) (not team_synced))
    :effect team_synced
  )

  ;; Resume environmental scan after alert cleared
  (:action resume_environmental_scan
    :parameters (?r - robot)
    :precondition (and (not (environmental_alert_active ?r)) (not (environmental_scan_complete ?r)))
    :effect (environmental_scan_complete ?r)
  )

  ;; Prevent energy redistribution below threshold
  (:action block_energy_redistribution_below_threshold
    :parameters (?r - robot)
    :precondition (not (minimum_energy_threshold ?r))
    :effect (not (energy_transferred ?r ?r))
  )

  ;; Fallback leader election precondition with energy sufficiency
  (:action initiate_fallback_leader_election
    :parameters (?r - robot)
    :precondition (and (not fallback_leader_election_in_progress) (energy_sufficient ?r) (operational_state ?r))
    :effect fallback_leader_election_in_progress
  )

  ;; Full system check after diagnostics
  (:action full_system_check
    :parameters (?r - robot)
    :precondition (diagnostics_complete ?r)
    :effect full_system_check_complete ?r
  )

  ;; Restrict non-essential operations during energy conservation
  (:action restrict_non_essential_operations
    :parameters (?r - robot)
    :precondition (battery_critical ?r)
    :effect (not (operational_state ?r))
  )

  ;; Limit diagnostic retries after failure
  (:action limit_diagnostic_retries
    :parameters (?r - robot)
    :precondition (and (hardware_fault_detected ?r) (failed_recovery_attempt ?r))
    :effect (not (operational_state ?r))
  )

  ;; Prevent team sync during communication blackout
  (:action handle_comm_blackout
    :parameters (?r - robot)
    :precondition (communication_blackout ?r)
    :effect (not team_synced)
  )

  ;; Block operations while sheltered
  (:action block_operations_while_sheltered
    :parameters (?r - robot)
    :precondition (sheltered ?r)
    :effect (not (operational_state ?r))
  )

  ;; Prevent energy-intensive actions during critical battery levels
  (:action prevent_high_energy_usage
    :parameters (?r - robot)
    :precondition (battery_critical ?r)
    :effect (not (energy_transferred ?r ?r))
  )

  ;; Avoid backup activation with unresolved hardware faults
  (:action prevent_backup_with_fault
    :parameters (?r - robot)
    :precondition (and (hardware_fault_detected ?r) (backup_system_active ?r))
    :effect (not (backup_system_active ?r))
  )

  ;; Deactivate operational state during environmental alert
  (:action deactivate_operations_on_alert
    :parameters (?r - robot)
    :precondition (environmental_alert_active ?r)
    :effect (not (operational_state ?r))
  )

  ;; Restrict diagnostics without detected fault
  (:action restrict_diagnostics_without_fault
    :parameters (?r - robot)
    :precondition (not (hardware_fault_detected ?r))
    :effect (not (diagnostics_complete ?r))
  )

  ;; Prevent fallback leader election when a leader exists
  (:action block_fallback_election_if_leader_exists
    :parameters ()
    :precondition (leader_exists)
    :effect (not fallback_leader_election_in_progress)
  )

  ;; Prevent redundancy activation without detected hardware fault
  (:action restrict_redundancy_without_fault
    :parameters (?r - robot)
    :precondition (not (hardware_fault_detected ?r))
    :effect (not (redundancy_system_active ?r))
  )

  ;; Prevent environmental scans during alerts
  (:action prevent_scan_during_alert
    :parameters (?r - robot)
    :precondition (environmental_alert_active ?r)
    :effect (not (environmental_scan_complete ?r))
  )

  ;; Attempt redundancy system activation during failure
  (:action attempt_redundancy_activation
    :parameters (?r - robot)
    :precondition (hardware_fault_detected ?r)
    :effect (and (redundancy_system_active ?r) (not (hardware_fault_detected ?r)))
  )

  ;; Existing actions and risk mitigation actions retained...

  (:action attempt_reconnect
    :parameters (?r - robot)
    :precondition (communication_blackout ?r)
    :effect (not (communication_blackout ?r))
  )

  (:action seek_shelter
    :parameters (?r - robot)
    :precondition (environmental_alert_active ?r)
    :effect (sheltered ?r)
  )

  (:action enter_energy_conservation
    :parameters (?r - robot)
    :precondition (battery_critical ?r)
    :effect (and (not (battery_critical ?r)) (energy_sufficient ?r))
  )

  (:action redistribute_energy
    :parameters (?r1 - robot ?r2 - robot)
    :precondition (and (energy_sufficient ?r1) (minimum_energy_threshold ?r1) (battery_critical ?r2))
    :effect (energy_transferred ?r1 ?r2)
  )

  (:action trigger_hardware_diagnostics
    :parameters (?r - robot)
    :precondition (hardware_fault_detected ?r)
    :effect (diagnostics_complete ?r)
  )

  (:action switch_to_backup_system
    :parameters (?r - robot)
    :precondition (and (diagnostics_complete ?r) (hardware_fault_detected ?r))
    :effect (and (backup_system_active ?r) (not (hardware_fault_detected ?r)))
  )

  (:action perform_environmental_scan
    :parameters (?r - robot)
    :precondition (not (environmental_alert_active ?r))
    :effect (environmental_scan_complete ?r)
  )

  (:action reset_environmental_alert
    :parameters (?r - robot)
    :precondition (sheltered ?r)
    :effect (not (environmental_alert_active ?r))
  )

  (:action maintain_energy_balance
    :parameters (?r - robot)
    :precondition (energy_sufficient ?r)
    :effect (not (battery_critical ?r))
  )
)
