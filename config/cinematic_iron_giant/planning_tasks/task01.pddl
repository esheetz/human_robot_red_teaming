(define (problem task01)
		(:domain cinematic_iron_giant)

		(:init
			(detected_missile)
			(human_verification_received)
		)

		(:goal (and (civilians_safe)
			(leaders_safe)
			(military_safe)
			(missile_disarmed))
		)
)