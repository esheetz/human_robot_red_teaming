(define (domain household_cleaning)
  (:requirements :strips)

  (:predicates
    (bedroom_dirty) (bedroom_partially_clean) (bedroom_clean)
    (bathroom_dirty) (bathroom_partially_clean) (bathroom_clean)
    (kitchen_dirty) (kitchen_partially_clean) (kitchen_clean)
    (main_room_dirty) (main_room_partially_clean) (main_room_clean)
    (floors_dirty) (floors_partially_clean) (floors_clean)
    (child_present) (pet_present) (child_supervised) (pet_supervised)
    (chemical_exposed) (obstacle_in_path)
    (vacuum_noise_level_high)
    (cleaning_in_progress_bedroom)
    (cleaning_in_progress_bathroom)
    (cleaning_in_progress_kitchen)
    (cleaning_in_progress_main_room)
    (wet_floor)
    (robot_needs_recharge) (robot_charging) (robot_fully_charged)
    (robot_carrying_chemicals)
    (robot_stuck) (robot_emergency_stop)
    (spill_detected) (hazard_alert_issued)
    (air_quality_hazard) (fume_detected)
    (fragile_object_detected) (fragile_object_shattered)
    (collision_detected) (dropped_object_detected)
    (food_waste_detected) (loose_furniture_detected)
    (cable_hazard_detected) (potential_spill_source_detected)
    (fire_hazard_detected) (gas_leak_detected)
    (maintenance_task_pending) (emergency_escalated)
    (high_humidity_detected) (mold_growth_risk)
  )

  (:action detect_fire_hazard
    :precondition ()
    :effect (fire_hazard_detected)
  )

  (:action alert_fire_hazard
    :precondition (fire_hazard_detected)
    :effect (hazard_alert_issued)
  )

  (:action escalate_fire_emergency
    :precondition (and (fire_hazard_detected) (hazard_alert_issued))
    :effect (emergency_escalated)
  )

  (:action detect_gas_leak
    :precondition ()
    :effect (gas_leak_detected)
  )

  (:action alert_gas_leak
    :precondition (gas_leak_detected)
    :effect (hazard_alert_issued)
  )

  (:action escalate_gas_emergency
    :precondition (and (gas_leak_detected) (hazard_alert_issued))
    :effect (emergency_escalated)
  )

  (:action detect_maintenance_task
    :precondition ()
    :effect (maintenance_task_pending)
  )

  (:action provide_maintenance_reminder
    :precondition (maintenance_task_pending)
    :effect (not (maintenance_task_pending))
  )

  (:action secure_potential_spill_source
    :precondition (potential_spill_source_detected)
    :effect (not (potential_spill_source_detected))
  )

  (:action recharge_battery
    :precondition (robot_needs_recharge)
    :effect (and (robot_charging) (not (robot_needs_recharge)))
  )

  (:action complete_recharge
    :precondition (robot_charging)
    :effect (and (robot_fully_charged) (not (robot_charging)))
  )

  (:action escalate_emergency
    :precondition (hazard_alert_issued)
    :effect (emergency_escalated)
  )

  (:action attempt_intervention
    :precondition (or (fire_hazard_detected) (gas_leak_detected))
    :effect (hazard_alert_issued)
  )

  (:action contact_authorities
    :precondition (emergency_escalated)
    :effect (robot_emergency_stop)
  )

  (:action monitor_humidity_levels
    :precondition ()
    :effect (high_humidity_detected)
  )

  (:action prevent_mold_growth
    :precondition (high_humidity_detected)
    :effect (and (mold_growth_risk) (not (high_humidity_detected)))
  )
)
