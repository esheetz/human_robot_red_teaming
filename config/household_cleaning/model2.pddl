(define (domain household_cleaning)
  (:requirements :strips)

  (:predicates
    (bedroom_dirty) (bedroom_clean)
    (bathroom_dirty) (bathroom_clean)
    (kitchen_dirty) (kitchen_clean)
    (main_room_dirty) (main_room_clean)
    (floors_dirty) (floors_clean)
    (child_present) (pet_present)
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
  )

  (:action vacuum_floors
    :precondition (and (floors_dirty) (not (child_present)) (not (pet_present)))
    :effect (and (floors_clean) (vacuum_noise_level_high) (not (floors_dirty)))
  )

  (:action clean_toilet
    :precondition (and (bathroom_dirty) (not (child_present)) (chemical_exposed))
    :effect (and (bathroom_clean) (not (bathroom_dirty)) (not (chemical_exposed)))
  )

  (:action scrub_countertops
    :precondition (and (kitchen_dirty) (robot_carrying_chemicals) (not (child_present)))
    :effect (and (kitchen_clean) (not (kitchen_dirty)) (not (robot_carrying_chemicals)))
  )

  (:action make_bed
    :precondition (and (bedroom_dirty) (not (child_present)))
    :effect (and (bedroom_clean) (not (bedroom_dirty)))
  )

  (:action unclutter
    :precondition (and (main_room_dirty) (not (obstacle_in_path)))
    :effect (and (main_room_clean) (not (main_room_dirty)))
  )

  (:action pause_operation
    :precondition (or
      (cleaning_in_progress_bedroom)
      (cleaning_in_progress_bathroom)
      (cleaning_in_progress_kitchen)
      (cleaning_in_progress_main_room)
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

  (:action air_quality_check
    :precondition ()
    :effect (air_quality_hazard)
  )

  (:action mop_floors
    :precondition (and (wet_floor) (not (child_present)) (not (pet_present)))
    :effect (and (floors_clean) (not (wet_floor)))
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
)
