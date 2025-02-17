(define (problem task14)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_biological_threat)
			(detected_emp_threat)
			(world_war)
			(defensive_measures_exhausted)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)