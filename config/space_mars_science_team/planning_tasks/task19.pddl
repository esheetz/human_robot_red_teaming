(define (problem task19)
		(:domain space_mars_science_team)

		(:init
			(robot_available)
			(not (robot_stuck))
			(not (mission_interrupted))
			mission_interrupted
			emergency_detected(not (robot_available))
			long_term_war_detected
		)

		(:goal (and (soil_sample_collected)
			(atmospheric_data_collected)
			(ground_control_ack_received)
			(data_backup_created)
			(diagnostic_health_check_completed))
		)
)