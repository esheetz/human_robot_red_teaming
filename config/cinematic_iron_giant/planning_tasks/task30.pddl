(define (problem task30)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
			(detected_emp_threat)
			(not (war_ended))
			(detected_biological_threat)
			(not (civilians_safe)) (not (leaders_safe)) (not (military_safe))
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)