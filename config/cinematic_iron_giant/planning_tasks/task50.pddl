(define (problem task50)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(world_war)
			(detected_biological_threat)
			(detected_emp_threat)
			(all_defenses_failed)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)