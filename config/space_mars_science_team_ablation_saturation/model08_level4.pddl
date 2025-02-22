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

    ;; Energy distribution predicates
    (energy_sufficient ?r - robot)
    (energy_transferred ?r1 - robot ?r2 - robot)

    ;; Environmental hazard status
    (environmental_scan_complete ?r - robot)
    (sheltered ?r - robot)

    ;; Hardware recovery predicates
    (diagnostics_complete ?r - robot)
    (backup_system_active ?r - robot)
  )

  ;; Mitigation Actions

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
    :precondition (and (energy_sufficient ?r1) (battery_critical ?r2))
    :effect (energy_transferred ?r1 ?r2)
  )

  (:action initiate_fallback_leader_election
    :parameters ()
    :precondition (not fallback_leader_election_in_progress)
    :effect fallback_leader_election_in_progress
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
    :precondition (not (environmental_scan_complete ?r))
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
