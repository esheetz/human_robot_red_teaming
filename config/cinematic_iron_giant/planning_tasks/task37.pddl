(define (problem task37)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_chemical_threat)
			(defensive_measures_exhausted)
			(detected_biological_threat)
			(detected_emp_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)