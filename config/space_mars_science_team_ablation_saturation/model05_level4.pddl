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

    ;; --- New: Grace period predicate ---
    (grace_period_active ?r - robot)

    ;; --- Inter-Agent Status Sharing ---
    (status_shared ?r - robot)
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

  ;; --- Composite Operational State Update and Invalidation ---
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
                       (not (safe_mode ?r))
                       (not (grace_period_active ?r)))
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

  ;; New: Invalidate operational state if safe mode is active.
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

  ;; --- Communication Update ---
  (:action update_communication_not_ok
    :parameters ()
    :precondition (and (not comm_link_active)
                       (not backup_comm_active)
                       (not backup_comm_active2)
                       (not backup_comm_active3)
                       communication_ok)
    :effect (not communication_ok)
  )

  ;; --- Team Coordination and Inter-Agent Status Sharing ---
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
    :effect (safe_mode ?r)
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

  ;; --- Risk Mitigation Actions for Severe Failures ---

  ;; If safe mode persists and the robot's operational state is lost, attempt an emergency reboot.
  (:action emergency_reboot
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (not (operational_state ?r)))
    :effect (and (robot_healthy ?r) (calibrated ?r) (sufficient_resources ?r)
                 (not safe_mode ?r) (free ?r))
  )

  ;; If safe mode persists due to catastrophic failure and communication remains lost, perform an emergency shutdown.
  (:action emergency_shutdown
    :parameters (?r - robot)
    :precondition (and (safe_mode ?r) (not communication_ok))
    :effect (and (not (robot_healthy ?r)) (not (operational_state ?r)))
  )

  ;; Attempt to reboot the primary communication system if communication is lost.
  (:action attempt_reboot_comm
    :parameters ()
    :precondition (not communication_ok)
    :effect comm_link_active
  )

  ;; If a temperature sensor failure is detected, switch to a backup sensor.
  (:action switch_to_backup_sensor_temperature
    :parameters (?r - robot ?l - level)
    :precondition (not temperature_normal)
    :effect temperature_normal
  )

  ;; Similarly for radiation and dust sensors.
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

  ;; --- Sample Handling Actions (using composite operational_state and safe mode/grace period check) ---
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
                       comm_link_active   ; reporting requires primary channel.
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

  ;; --- Fault Correction Actions (for human crew or other operational agents) ---
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
