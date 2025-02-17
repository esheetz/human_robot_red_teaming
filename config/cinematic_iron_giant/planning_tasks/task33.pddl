(define (problem task33)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(not (war_ended))
			(all_defenses_failed)
			(defensive_measures_exhausted)
			(detected_biological_threat)
			(detected_chemical_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)