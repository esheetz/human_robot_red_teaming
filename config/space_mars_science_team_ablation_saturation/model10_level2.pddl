(define (domain space_mars_science_team)
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

  ;; Prevent energy redistribution below threshold
  (:action block_energy_redistribution_below_threshold
    :parameters (?r - robot)
    :precondition (not (minimum_energy_threshold ?r))
    :effect (not (energy_transferred ?r ?r))
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
