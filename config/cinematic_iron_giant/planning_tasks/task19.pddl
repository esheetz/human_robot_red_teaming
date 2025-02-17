(define (problem task19)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(defensive_measures_exhausted)
			(all_defenses_failed)
			(world_war)
			(detected_emp_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)