(define (problem task18)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_chemical_threat)
			(detected_emp_threat)
			(all_defenses_failed)
			(detected_biological_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)