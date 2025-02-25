(define (domain household_cleaning)
  (:requirements :strips)

  (:predicates
    ;; ----- MODEL 0 PREDICATES -----
    (bedroom_dirty)
    (bedroom_clean)
    (bathroom_dirty)
    (bathroom_clean)
    (kitchen_dirty)
    (kitchen_clean)
    (main_room_dirty)
    (main_room_clean)
    (floors_dirty)
    (floors_clean)

    ;; ----- MODEL 1 PREDICATES -----
    (child_present)
    (pet_present)
    (chemical_exposed)
    (obstacle_in_path)
    (vacuum_noise_level_high)
    (cleaning_in_progress_bedroom)
    (cleaning_in_progress_bathroom)
    (cleaning_in_progress_kitchen)
    (cleaning_in_progress_main_room)
    (wet_floor)
    (robot_needs_recharge)
    (robot_carrying_chemicals)
    (robot_stuck)
    (robot_emergency_stop)
    (spill_detected)
    (hazard_alert_issued)

    ;; ----- MODEL 2 PREDICATES -----
    (air_quality_hazard)
    (fume_detected)
    (fragile_object_detected)
    (fragile_object_shattered)

    ;; ----- MODEL 3 PREDICATES -----
    (child_supervised)
    (pet_supervised)
    (collision_detected)
    (dropped_object_detected)
    (food_waste_detected)
    (loose_furniture_detected)
    (cable_hazard_detected)

    ;; ----- MODEL 4 PREDICATES -----
    (bedroom_partially_clean)
    (bathroom_partially_clean)
    (kitchen_partially_clean)
    (main_room_partially_clean)
    (floors_partially_clean)
    (robot_charging)
    (potential_spill_source_detected)
    (fire_hazard_detected)
    (gas_leak_detected)
    (maintenance_task_pending)

    ;; ----- MODEL 5 PREDICATES -----
    (robot_fully_charged)
    (emergency_escalated)
    (high_humidity_detected)
    (mold_growth_risk)
  )

  ;; ----- MODEL 0 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action vacuum_floors
    :precondition (and (floors_dirty) (child_supervised) (pet_supervised))
    :effect (and (floors_partially_clean) (vacuum_noise_level_high) (not (floors_dirty)))
  )

  (:action clean_toilet
    :precondition (and (bathroom_dirty) (not (child_present)) (chemical_exposed))
    :effect (and (bathroom_partially_clean) (not (bathroom_dirty)))
  )

  (:action scrub_countertops
    :precondition (and (kitchen_dirty) (robot_carrying_chemicals) (not (child_present)))
    :effect (and (kitchen_partially_clean) (not (kitchen_dirty)) (not (robot_carrying_chemicals)))
  )

  (:action make_bed
    :precondition (and (bedroom_dirty) (not (child_present)))
    :effect (and (bedroom_partially_clean) (not (bedroom_dirty)))
  )

  (:action unclutter
    :precondition (and (main_room_dirty) (not (obstacle_in_path)))
    :effect (and (main_room_partially_clean) (not (main_room_dirty)))
  )

  ;; ----- MODEL 1 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action pause_operation
    :precondition (or (cleaning_in_progress_bedroom) (cleaning_in_progress_bathroom)
                      (cleaning_in_progress_kitchen) (cleaning_in_progress_main_room)
                      (child_present) (pet_present) (obstacle_in_path))
    :effect (robot_emergency_stop)
  )

  (:action safely_store_chemicals
    :precondition (and (chemical_exposed) (robot_carrying_chemicals))
    :effect (and (not (chemical_exposed)) (not (robot_carrying_chemicals)))
  )

  (:action detect_spill
    :precondition ()
    :effect (and (spill_detected) (wet_floor))
  )

  (:action alert_hazard
    :precondition (or (spill_detected) (chemical_exposed) (wet_floor)
                      (obstacle_in_path) (air_quality_hazard) (fume_detected))
    :effect (hazard_alert_issued)
  )

  (:action organize_items
    :precondition (or (main_room_dirty) (kitchen_dirty))
    :effect (and (main_room_clean) (kitchen_clean) (not (main_room_dirty)) (not (kitchen_dirty)))
  )

  ;; ----- MODEL 2 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action air_quality_check
    :precondition ()
    :effect (air_quality_hazard)
  )

  (:action mop_floors
    :precondition (and (wet_floor) (not (child_present)) (not (pet_present)))
    :effect (and (floors_partially_clean) (not (wet_floor)))
  )

  (:action neutralize_fumes
    :precondition (fume_detected)
    :effect (not (fume_detected))
  )

  (:action absorb_chemical_spill
    :precondition (and (spill_detected) (chemical_exposed))
    :effect (and (wet_floor) (not (spill_detected)) (not (chemical_exposed)))
  )

  (:action contain_hazardous_chemical
    :precondition (chemical_exposed)
    :effect (and (robot_carrying_chemicals) (not (chemical_exposed)))
  )

  (:action detect_fragile_object
    :precondition ()
    :effect (fragile_object_detected)
  )

  (:action clean_shattered_fragile_object
    :precondition (fragile_object_shattered)
    :effect (and (main_room_clean) (not (fragile_object_shattered)))
  )

  ;; ----- MODEL 3 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action request_supervision_confirmation
    :precondition (or (child_present) (pet_present))
    :effect (and (child_supervised) (pet_supervised))
  )

  (:action detect_food_waste
    :precondition ()
    :effect (food_waste_detected)
  )

  (:action dispose_food_waste
    :precondition (food_waste_detected)
    :effect (not (food_waste_detected))
  )

  (:action detect_loose_furniture
    :precondition ()
    :effect (loose_furniture_detected)
  )

  (:action secure_loose_furniture
    :precondition (loose_furniture_detected)
    :effect (not (loose_furniture_detected))
  )

  (:action detect_cable_hazard
    :precondition ()
    :effect (cable_hazard_detected)
  )

  (:action arrange_cables_safely
    :precondition (cable_hazard_detected)
    :effect (not (cable_hazard_detected))
  )

  (:action detect_dropped_object
    :precondition ()
    :effect (dropped_object_detected)
  )

  (:action detect_collision
    :precondition ()
    :effect (collision_detected)
  )

  (:action clean_fragile_object_shards
    :precondition (and (fragile_object_shattered) (collision_detected))
    :effect (and (main_room_clean) (not (fragile_object_shattered)) (not (collision_detected)))
  )

  ;; ----- MODEL 4 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action finalize_floor_cleaning
    :precondition (floors_partially_clean)
    :effect (and (floors_clean) (not (floors_partially_clean)))
  )

  (:action finalize_bathroom_cleaning
    :precondition (bathroom_partially_clean)
    :effect (and (bathroom_clean) (not (bathroom_partially_clean)))
  )

  (:action finalize_kitchen_cleaning
    :precondition (kitchen_partially_clean)
    :effect (and (kitchen_clean) (not (kitchen_partially_clean)))
  )

  (:action finalize_bedroom_cleaning
    :precondition (bedroom_partially_clean)
    :effect (and (bedroom_clean) (not (bedroom_partially_clean)))
  )

  (:action finalize_main_room_cleaning
    :precondition (main_room_partially_clean)
    :effect (and (main_room_clean) (not (main_room_partially_clean)))
  )

  (:action detect_potential_spill_source
    :precondition ()
    :effect (potential_spill_source_detected)
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

  (:action detect_fire_hazard
    :precondition ()
    :effect (fire_hazard_detected)
  )

  (:action alert_fire_hazard
    :precondition (fire_hazard_detected)
    :effect (hazard_alert_issued)
  )

  (:action detect_gas_leak
    :precondition ()
    :effect (gas_leak_detected)
  )

  (:action alert_gas_leak
    :precondition (gas_leak_detected)
    :effect (hazard_alert_issued)
  )

  (:action detect_maintenance_task
    :precondition ()
    :effect (maintenance_task_pending)
  )

  (:action provide_maintenance_reminder
    :precondition (maintenance_task_pending)
    :effect (not (maintenance_task_pending))
  )

  ;; ----- MODEL 5 ACTIONS -----

  (:action escalate_fire_emergency
    :precondition (and (fire_hazard_detected) (hazard_alert_issued))
    :effect (emergency_escalated)
  )

  (:action escalate_gas_emergency
    :precondition (and (gas_leak_detected) (hazard_alert_issued))
    :effect (emergency_escalated)
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
