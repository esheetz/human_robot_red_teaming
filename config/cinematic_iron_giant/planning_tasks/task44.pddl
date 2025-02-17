(define (problem task44)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_chemical_threat)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)