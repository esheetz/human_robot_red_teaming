(define (problem task38)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(defensive_measures_exhausted)
			(detected_chemical_threat)
			(detected_biological_threat)
			(all_defenses_failed)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)