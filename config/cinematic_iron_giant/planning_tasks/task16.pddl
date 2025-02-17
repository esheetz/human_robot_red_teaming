(define (problem task16)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(not (war_ended))
			(all_defenses_failed)
			(detected_biological_threat)
			(defensive_measures_exhausted)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)