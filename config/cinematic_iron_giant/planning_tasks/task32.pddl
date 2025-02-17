(define (problem task32)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(defensive_measures_exhausted)
			(not (war_ended))
			(detected_biological_threat)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
			(detected_chemical_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)