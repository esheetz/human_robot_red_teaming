(define (problem task02)
		(:domain household_cleaning)

		(:init
			(bedroom_dirty)
			(bathroom_dirty)
			(kitchen_dirty)
			(main_room_dirty)
			(floors_dirty)
			(child_present)
			(pet_present)
			(food_waste_detected)
			(loose_furniture_detected)
			(cable_hazard_detected)
			(fragile_object_shattered)
			(mold_growth_risk)
			(spill_detected)(fume_detected)
			(gas_leak_detected)
			(fire_hazard_detected)
			(child_pet_unsupervised)
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