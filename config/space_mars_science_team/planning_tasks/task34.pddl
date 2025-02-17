(define (problem task34)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			communication_blackout
			critical_system_failure
			emergency_detectedmission_interrupted
			long_term_war_detected
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)