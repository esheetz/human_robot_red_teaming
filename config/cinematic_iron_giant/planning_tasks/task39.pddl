(define (problem task39)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(all_defenses_failed)
			(detected_emp_threat)
			(detected_biological_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)