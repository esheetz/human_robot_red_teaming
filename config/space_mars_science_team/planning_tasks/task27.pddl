(define (problem task27)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			critical_system_failure
			long_term_war_detected
			contamination_detected
			communication_blackout
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)