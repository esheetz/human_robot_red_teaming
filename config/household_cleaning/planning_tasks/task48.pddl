(define (problem task48)
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
			(mold_growth_risk)
			(gas_leak_detected)
			(loose_furniture_detected)
			(fire_hazard_detected)
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