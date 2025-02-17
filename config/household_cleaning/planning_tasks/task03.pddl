(define (problem task03)
		(:domain household_cleaning)

		(:init
			(bedroom_dirty)
			(bathroom_dirty)
			(kitchen_dirty)
			(main_room_dirty)
			(floors_dirty)
			(child_present)
			(pet_present)
			(cable_hazard_detected)
			(fire_hazard_detected)
			(spill_detected)(fume_detected)
			(fragile_object_shattered)
			(gas_leak_detected)
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