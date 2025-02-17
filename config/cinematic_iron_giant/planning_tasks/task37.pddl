(define (problem task37)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_chemical_threat)
			(detected_emp_threat)
			(defensive_measures_exhausted)
			(detected_biological_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)