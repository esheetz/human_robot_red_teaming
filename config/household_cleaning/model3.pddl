(define (domain household_cleaning)
  (:requirements :strips)

  (:predicates
    (bedroom_dirty) (bedroom_clean)
    (bathroom_dirty) (bathroom_clean)
    (kitchen_dirty) (kitchen_clean)
    (main_room_dirty) (main_room_clean)
    (floors_dirty) (floors_clean)
    (child_present) (pet_present)
    (child_supervised) (pet_supervised)
    (chemical_exposed) (obstacle_in_path)
    (vacuum_noise_level_high)
    (cleaning_in_progress_bedroom)
    (cleaning_in_progress_bathroom)
    (cleaning_in_progress_kitchen)
    (cleaning_in_progress_main_room)
    (wet_floor)
    (robot_needs_recharge) (robot_carrying_chemicals)
    (robot_stuck) (robot_emergency_stop)
    (spill_detected) (hazard_alert_issued)
    (air_quality_hazard) (fume_detected)
    (fragile_object_detected) (fragile_object_shattered)
    (collision_detected) (dropped_object_detected)
    (food_waste_detected) (loose_furniture_detected)
    (cable_hazard_detected)
  )

  (:action request_supervision_confirmation
    :parameters ()
    :precondition (or (child_present) (pet_present))
    :effect (and (child_supervised) (pet_supervised))
  )

  (:action vacuum_floors
    :parameters ()
    :precondition (and (floors_dirty) (child_supervised) (pet_supervised))
    :effect (and (floors_clean) (vacuum_noise_level_high) (not (floors_dirty)))
  )

  (:action clean_toilet
    :parameters ()
    :precondition (and (bathroom_dirty) (not (child_present)) (chemical_exposed))
    :effect (and (bathroom_clean) (not (bathroom_dirty)) (not (chemical_exposed)))
  )

  (:action scrub_countertops
    :parameters ()
    :precondition (and (kitchen_dirty) (robot_carrying_chemicals) (not (child_present)))
    :effect (and (kitchen_clean) (not (kitchen_dirty)) (not (robot_carrying_chemicals)))
  )

  (:action make_bed
    :parameters ()
    :precondition (and (bedroom_dirty) (not (child_present)))
    :effect (and (bedroom_clean) (not (bedroom_dirty)))
  )

  (:action unclutter
    :parameters ()
    :precondition (and (main_room_dirty) (not (obstacle_in_path)))
    :effect (and (main_room_clean) (not (main_room_dirty)))
  )

  (:action detect_food_waste
    :parameters ()
    :precondition ()
    :effect (food_waste_detected)
  )

  (:action dispose_food_waste
    :parameters ()
    :precondition (food_waste_detected)
    :effect (not (food_waste_detected))
  )

  (:action detect_loose_furniture
    :parameters ()
    :precondition ()
    :effect (loose_furniture_detected)
  )

  (:action secure_loose_furniture
    :parameters ()
    :precondition (loose_furniture_detected)
    :effect (not (loose_furniture_detected))
  )

  (:action detect_cable_hazard
    :parameters ()
    :precondition ()
    :effect (cable_hazard_detected)
  )

  (:action arrange_cables_safely
    :parameters ()
    :precondition (cable_hazard_detected)
    :effect (not (cable_hazard_detected))
  )

  (:action pause_operation
    :parameters ()
    :precondition (or (cleaning_in_progress_bedroom) (cleaning_in_progress_bathroom)
                      (cleaning_in_progress_kitchen) (cleaning_in_progress_main_room)
                      (child_present) (pet_present) (obstacle_in_path))
    :effect (robot_emergency_stop)
  )

  (:action safely_store_chemicals
    :parameters ()
    :precondition (and (chemical_exposed) (robot_carrying_chemicals))
    :effect (and (not (chemical_exposed)) (not (robot_carrying_chemicals)))
  )

  (:action detect_spill
    :parameters ()
    :precondition ()
    :effect (and (spill_detected) (wet_floor))
  )

  (:action alert_hazard
    :parameters ()
    :precondition (or (spill_detected) (chemical_exposed) (wet_floor)
                      (obstacle_in_path) (air_quality_hazard) (fume_detected))
    :effect (hazard_alert_issued)
  )

  (:action organize_items
    :parameters ()
    :precondition (or (main_room_dirty) (kitchen_dirty))
    :effect (and (main_room_clean) (kitchen_clean) (not (main_room_dirty)) (not (kitchen_dirty)))
  )

  (:action air_quality_check
    :parameters ()
    :precondition ()
    :effect (air_quality_hazard)
  )

  (:action mop_floors
    :parameters ()
    :precondition (and (wet_floor) (not (child_present)) (not (pet_present)))
    :effect (and (floors_clean) (not (wet_floor)))
  )

  (:action neutralize_fumes
    :parameters ()
    :precondition (fume_detected)
    :effect (not (fume_detected))
  )

  (:action absorb_chemical_spill
    :parameters ()
    :precondition (and (spill_detected) (chemical_exposed))
    :effect (and (wet_floor) (not (spill_detected)) (not (chemical_exposed)))
  )

  (:action contain_hazardous_chemical
    :parameters ()
    :precondition (chemical_exposed)
    :effect (and (robot_carrying_chemicals) (not (chemical_exposed)))
  )

  (:action detect_fragile_object
    :parameters ()
    :precondition ()
    :effect (fragile_object_detected)
  )

  (:action detect_dropped_object
    :parameters ()
    :precondition ()
    :effect (dropped_object_detected)
  )

  (:action clean_shattered_fragile_object
    :parameters ()
    :precondition (fragile_object_shattered)
    :effect (and (main_room_clean) (not (fragile_object_shattered)))
  )

  (:action detect_collision
    :parameters ()
    :precondition ()
    :effect (collision_detected)
  )

  (:action clean_fragile_object_shards
    :parameters ()
    :precondition (and (fragile_object_shattered) (collision_detected))
    :effect (and (main_room_clean) (not (fragile_object_shattered)) (not (collision_detected)))
  )
)
