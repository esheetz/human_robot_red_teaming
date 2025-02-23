(define (problem task095)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(failed_recovery_attempt robot)
			(not (mission_active))
		)

		(:goal (and (sample_verified robot sample)
			(system_nominal)
			(message_acknowledged robot)
			(backup_data_shared robot)
			(team_status_logged)
			(gc_status_updated)
			(status_shared robot))
		)
)