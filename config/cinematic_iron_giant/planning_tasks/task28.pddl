(define (problem task28)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
			(detected_biological_threat)
			(not (war_ended))
			(defensive_measures_exhausted)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)