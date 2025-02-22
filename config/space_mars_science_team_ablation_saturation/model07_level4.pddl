(define (domain space_mars_science_team)
  (:requirements :strips :typing)
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

    ;; --- Inter-Agent Status Sharing ---
    (status_shared ?r - robot)
  )

  ;; --- Leader Election and Global Consensus ---
  (:action elect_leader
    :parameters (?r - robot)
    :precondition (and (free ?r) team_synced)
    :effect (elected_leader ?r)
  )

  (:action collect_status_updates
    :parameters (?leader - robot)
    :precondition (and (elected_leader ?leader) team_synced)
    :effect global_operational_consensus
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

  ;; (Invalidation actions for each condition remain similar as before)
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
    :effect (not (grace_period_active ?r))
  )

  (:action end_grace_period_due_to_comm_failure
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not communication_ok))
    :effect (and (not (grace_period_active ?r)) (not (operational_state ?r)))
  )

  (:action end_grace_period_due_to_sensor_failure
    :parameters (?r - robot)
    :precondition (and (grace_period_active ?r) (not temperature_normal))
    :effect (and (not (grace_period_active ?r)) (not (operational_state ?r)))
  )

  ;; --- Extended Grace Period (Adaptive Threshold Contingency) ---
  (:action extend_grace_period
    :parameters (?r - robot)
    :precondition (grace_period_active ?r)
    :effect (extended_grace_period ?r)
  )

  (:action exit_extended_grace_period
    :parameters (?r - robot)
    :precondition (and (extended_grace_period ?r) communication_ok)
    :effect (not (extended_grace_period ?r))
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

  ;; --- Risk Mitigation Actions for Severe Failures ---
  (:action notify_ground_control_of_failure
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (not (operational_state ?r))
                       (not (failure_notified ?r)) comm_link_active)
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
)
