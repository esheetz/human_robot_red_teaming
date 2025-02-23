(define (problem task063)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(not (system_nominal))
			(sensor_fault_detected robot)
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