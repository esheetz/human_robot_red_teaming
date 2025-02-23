(define (problem task007)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(not (full_system_check_complete robot))
			(sensor_fault_detected robot)
			(not (system_nominal))
			(failed_recovery_attempt robot)
			(not (diagnostics_complete robot))
			(communication_blackout robot)
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