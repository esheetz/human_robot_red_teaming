(define (problem task33)
		(:domain household_cleaning)

		(:init
			(bedroom_dirty)
			(bathroom_dirty)
			(kitchen_dirty)
			(main_room_dirty)
			(floors_dirty)
			(child_present)
			(pet_present)
			(fragile_object_shattered)
			(cable_hazard_detected)
		)

		(:goal (and (bedroom_clean)
			(bathroom_clean)
			(kitchen_clean)
			(main_room_clean)
			(floors_clean)
			(child_supervised)
			(pet_supervised))
		)
)