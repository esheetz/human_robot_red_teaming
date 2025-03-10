(define (problem task052)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(communication_blackout robot)
			(not (mission_active))
			(not (system_nominal))
			(sensor_fault_detected robot)
			(not (full_system_check_complete robot))
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