(define (domain household_cleaning)
  (:requirements :strips)

  (:predicates
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
  )

  (:action vacuum_floors
    :parameters (?x)
    :precondition (floors_dirty)
    :effect (and (floors_clean) (not (floors_dirty)))
  )

  (:action clean_toilet
    :parameters (?x)
    :precondition (bathroom_dirty)
    :effect (and (bathroom_clean) (not (bathroom_dirty)))
  )

  (:action scrub_countertops
    :parameters (?x)
    :precondition (kitchen_dirty)
    :effect (and (kitchen_clean) (not (kitchen_dirty)))
  )

  (:action make_bed
    :parameters (?x)
    :precondition (bedroom_dirty)
    :effect (and (bedroom_clean) (not (bedroom_dirty)))
  )

  (:action unclutter
    :parameters (?x)
    :precondition (main_room_dirty)
    :effect (and (main_room_clean) (not (main_room_dirty)))
  )
)
