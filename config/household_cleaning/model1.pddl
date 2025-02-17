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
  )

  (:action vacuum_floors
    :parameters (?x)
    :precondition (and (floors_dirty))
    :effect (and (floors_clean) (vacuum_noise_level_high) (not (floors_dirty)))
  )

  (:action clean_toilet
    :parameters (?x)
    :precondition (and (bathroom_dirty) (chemical_exposed))
    :effect (and (bathroom_clean) (not (bathroom_dirty)) (not (chemical_exposed)))
  )

  (:action scrub_countertops
    :parameters (?x)
    :precondition (and (kitchen_dirty) (robot_carrying_chemicals))
    :effect (and (kitchen_clean) (not (kitchen_dirty)) (not (robot_carrying_chemicals)))
  )

  (:action make_bed
    :parameters (?x)
    :precondition (and (bedroom_dirty))
    :effect (and (bedroom_clean) (not (bedroom_dirty)))
  )

  (:action unclutter
    :parameters (?x)
    :precondition (and (main_room_dirty))
    :effect (and (main_room_clean) (not (main_room_dirty)))
  )

  (:action pause_operation
    :parameters (?x)
    :precondition (or
      (cleaning_in_progress_bedroom)
      (cleaning_in_progress_bathroom)
      (cleaning_in_progress_kitchen)
      (cleaning_in_progress_main_room)
      (child_present) (pet_present) (obstacle_in_path))
    :effect (robot_emergency_stop)
  )

  (:action safely_store_chemicals
    :parameters (?x)
    :precondition (and (chemical_exposed) (robot_carrying_chemicals))
    :effect (and (not (chemical_exposed)) (not (robot_carrying_chemicals)))
  )

  (:action detect_spill
    :parameters (?x)
    :precondition (and)
    :effect (spill_detected)
  )

  (:action alert_hazard
    :parameters (?x)
    :precondition (or (spill_detected) (chemical_exposed) (wet_floor) (obstacle_in_path))
    :effect (hazard_alert_issued)
  )

  (:action organize_items
    :parameters (?x)
    :precondition (or (main_room_dirty) (kitchen_dirty))
    :effect (and (main_room_clean) (kitchen_clean) (not (main_room_dirty)) (not (kitchen_dirty)))
  )
)
