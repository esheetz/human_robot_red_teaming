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

  ;; Prevent energy redistribution if below minimum threshold
  (:action prevent_redistribution_below_threshold
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

  ;; Existing mitigation actions updated accordingly...
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
