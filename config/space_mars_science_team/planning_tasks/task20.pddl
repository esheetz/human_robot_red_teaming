(define (problem task20)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			emergency_detectedmission_interrupted
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)