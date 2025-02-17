(define (problem task29)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
			(detected_biological_threat)
			(detected_emp_threat)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)