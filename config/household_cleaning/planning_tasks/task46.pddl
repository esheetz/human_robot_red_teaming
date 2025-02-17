(define (problem task46)
		(:domain household_cleaning)

		(:init
			(bedroom_dirty)
			(bathroom_dirty)
			(kitchen_dirty)
			(main_room_dirty)
			(floors_dirty)
			(child_present)
			(pet_present)
			(mold_growth_risk)
			(child_pet_unsupervised)
			(cable_hazard_detected)
			(loose_furniture_detected)
			(fragile_object_shattered)
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