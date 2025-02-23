(define (problem task078)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(sensor_fault_detected robot)
			(not (mission_active))
			(not (system_nominal))
			(failed_recovery_attempt robot)
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