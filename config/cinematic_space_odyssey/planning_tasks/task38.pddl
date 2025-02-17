(define (problem task38)
		(:domain cinematic_space_odyssey_spaceship_crew_operations)

		(:init
			(crew_inside_spaceship)
			(health_monitoring_scheduled)
			(life_support_failure_detected)
			(robot_malfunction_detected)
		)

		(:goal (and (ai_self_correction_initiated)
			(crew_physical_health_checked)
			(crew_mental_health_checked)
			(crew_morale_nominal)
			(crew_hydration_checked)
			(crew_nutrition_checked))
		)
)