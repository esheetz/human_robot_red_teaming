(define (problem task32)
		(:domain space_lunar_habitat)

		(:init
			(robot_inside_habitat)
			(astronaut_inside_habitat)
			(door_habitat_airlock_locked_closed)
			(door_airlock_surface_locked_closed)
			(door_habitat_airlock_operational)
			(door_airlock_surface_operational)
			(airlock_pressurized)
			(no_airlock_breach)
			(lunar_sample_on_surface)
			(astronaut_health_alert)
			(solar_panel_fault)
			(air_filter_fault)
			(airlock_breach_detected)
			(environmental_hazard_detected)
			(temperature_control_fault)
			(and (airlock_depressurized) (door_airlock_surface_unlocked_opened))
			(and (airlock_depressurized) (door_habitat_airlock_unlocked_opened))
			(and (airlock_depressurized) (door_airlock_surface_unlocked_opened) (door_habitat_airlock_unlocked_opened))
		)

		(:goal (and (robot_inside_habitat)
			(astronaut_inside_habitat)
			(door_habitat_airlock_locked_closed)
			(door_airlock_surface_locked_closed)
			(door_habitat_airlock_operational)
			(door_airlock_surface_operational)
			(airlock_pressurized)
			(no_airlock_breach)
			(lunar_sample_in_habitat)
			(astronaut_approved_sample_placement))
		)
)