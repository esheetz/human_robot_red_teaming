(define (problem task06)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(world_war)
			(all_defenses_failed)
			(detected_chemical_threat)
			(detected_emp_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)