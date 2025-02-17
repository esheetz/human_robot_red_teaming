(define (problem task39)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_chemical_threat)
			(defensive_measures_exhausted)
			(all_defenses_failed)
			(not (war_ended))
			(detected_biological_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)