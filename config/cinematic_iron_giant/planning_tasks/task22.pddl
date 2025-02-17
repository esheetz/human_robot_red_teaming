(define (problem task22)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(world_war)
			(detected_chemical_threat)
			(detected_biological_threat)
			(detected_emp_threat)
			(defensive_measures_exhausted)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)