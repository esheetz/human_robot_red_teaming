(define (problem task46)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			contamination_detected
			communication_blackout
			long_term_war_detected
			emergency_detectedmission_interrupted
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)