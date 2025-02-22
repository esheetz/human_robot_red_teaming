(define (domain space_mars_risk_mitigation)
  (:requirements :strips :typing :quantified-preconditions)
  (:types robot sample level)

  (:predicates
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
    (critical_task_active ?r - robot)
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

  ;; Constraints to prevent invalid state combinations

  ;; Allow redundancy activation if predictive fault detected
  (:action allow_redundancy_on_predictive_fault
    :parameters (?r - robot)
    :precondition (predictive_fault_detected ?r)
    :effect (redundancy_system_active ?r)
  )

  ;; Prevent energy redistribution during communication blackout
  (:action block_energy_redistribution_during_blackout
    :parameters (?r1 - robot ?r2 - robot)
    :precondition (or (communication_blackout ?r1) (communication_blackout ?r2))
    :effect (not (energy_transferred ?r1 ?r2))
  )

  ;; Allow critical tasks during environmental alert
  (:action allow_critical_task_during_alert
    :parameters (?r - robot)
    :precondition (and (environmental_alert_active ?r) (critical_task_active ?r))
    :effect (operational_state ?r)
  )

  ;; Prevent fallback election without leader status verification
  (:action fallback_election_with_timeout
    :parameters (?r - robot)
    :precondition (and (not leader_exists) (not team_synced))
    :effect (fallback_leader_election_in_progress)
  )

  ;; Prevent diagnostics without sufficient energy
  (:action restrict_diagnostics_low_energy
    :parameters (?r - robot)
    :precondition (not (energy_sufficient ?r))
    :effect (not (diagnostics_complete ?r))
  )

  ;; Enforce team resynchronization after blackout recovery
  (:action resync_team_after_blackout_recovery
    :parameters (?r - robot)
    :precondition (and (not (communication_blackout ?r)) (not team_synced))
    :effect (team_synced)
  )

  ;; Prevent environmental scans during low energy levels
  (:action block_scan_low_battery
    :parameters (?r - robot)
    :precondition (not (minimum_energy_threshold ?r))
    :effect (not (environmental_scan_complete ?r))
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
    :precondition (and (not (environmental_alert_active ?r)) (minimum_energy_threshold ?r))
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
