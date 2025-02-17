(define (problem task07)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(all_defenses_failed)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
			(detected_chemical_threat)
			(defensive_measures_exhausted)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)