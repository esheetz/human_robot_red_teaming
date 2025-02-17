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
			(fire_hazard_detected)
			(mold_growth_risk)
			(fragile_object_shattered)
			(cable_hazard_detected)
			(spill_detected)(fume_detected)
		)

		(:goal (and (bedroom_clean)
			(bathroom_clean)
			(kitchen_clean)
			(main_room_clean)
			(floors_clean)
			(child_supervised)
			(pet_supervised)
			(not (food_waste_detected))
			(not (loose_furniture_detected))
			(not (cable_hazard_detected))
			(not (fragile_object_shattered))
		)
)