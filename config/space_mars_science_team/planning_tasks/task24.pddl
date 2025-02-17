(define (problem task24)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			long_term_war_detected
			emergency_detectedmission_interrupted
			communication_blackout
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)