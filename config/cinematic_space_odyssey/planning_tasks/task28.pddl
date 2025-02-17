(define (problem task28)
		(:domain cinematic_space_odyssey)

		(:init
			(crew_inside_spaceship)
			(health_monitoring_scheduled)
			(not (crew_nutrition_checked))
			(not (crew_fatigue_monitored))
			(systems_malfunction_detected)
			(not (crew_morale_nominal))
			(not (crew_phyiscal_health_checked))(not (crew_mental_health_checked))
			(not (ai_self_correction_initiated))
			(not (crew_hydration_checked))
			(robot_malfunction_detected)
			(not (ai_rogue))
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