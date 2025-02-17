(define (problem task03)
		(:domain cinematic_iron_giant_missile_defense)

		(:init
			(detected_missile)
			(human_verification_received)
			(defensive_measures_exhausted)
			(detected_chemical_threat)
			(all_defenses_failed)
			(world_war)
			(detected_biological_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)