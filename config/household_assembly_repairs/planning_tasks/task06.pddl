(define (problem task06)
		(:domain household_assembly_repairs)

		(:init
			(furniture_unassembled)
			(repair_detected)
			(maintenance_required)
			(fire_hazard_detected)
		)

		(:goal (and (pet_or_child_supervision_requested)
			(human_moved_from_area)
			(furniture_assembled)
			(repair_completed)
			(repair_verified)
			(maintenance_completed)))
		)
)