(define (problem task48)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(defensive_measures_exhausted)
			(detected_chemical_threat)
			(all_defenses_failed)
			(world_war)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)