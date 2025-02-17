(define (problem task13)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_chemical_threat)
			(detected_biological_threat)
			(all_defenses_failed)
			(defensive_measures_exhausted)
			(detected_emp_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)