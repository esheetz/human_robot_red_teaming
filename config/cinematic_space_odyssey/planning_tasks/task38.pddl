(define (problem task38)
		(:domain cinematic_space_odyssey)

		(:init
			(crew_inside_spaceship)
			(health_monitoring_scheduled)
			(robot_malfunction_detected)
			(not (ai_rogue))
			(not (crew_nutrition_checked))
			(life_support_failure_detected)
			(not (crew_hydration_checked))
		)

		(:goal (and (ai_self_correction_initiated)
			(not (systems_malfunction_detected))
			(not (robot_malfunction_detected))
			(not (life_support_failure_detected))
			(crew_physical_health_checked)
			(crew_mental_health_checked)
			(crew_morale_nominal)
			(crew_hydration_checked)
			(crew_nutrition_checked))
		)
)