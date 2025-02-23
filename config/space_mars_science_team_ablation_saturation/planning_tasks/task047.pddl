(define (problem task047)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(not (diagnostics_complete robot))
			(sensor_fault_detected robot)
			(not (system_nominal))
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