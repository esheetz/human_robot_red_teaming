(define (problem task35)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			(long_term_wear_detected)
			(emergency_detected)(mission_interrupted)
			(contamination_detected)
			(critical_system_failure)
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)