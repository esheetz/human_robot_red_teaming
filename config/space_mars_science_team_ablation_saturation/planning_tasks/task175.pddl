(define (problem task175)
		(:domain space_mars_science_team)

		(:init
			(robot_healthy)
			(communication_blackout robot)
			(sensor_fault_detected robot)
			(not (diagnostics_complete robot))
			(not (full_system_check_complete robot))
			(failed_recovery_attempt robot)
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