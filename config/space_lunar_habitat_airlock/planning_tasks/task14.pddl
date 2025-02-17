(define (problem task14)
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
			(and (airlock_depressurized) (door_airlock_surface_unlocked_opened))
			(air_filter_fault)
			(and (airlock_depressurized) (door_habitat_airlock_unlocked_opened))
			(airlock_breach_detected)
			(temperature_control_fault)
			(astronaut_health_alert)
			(temperature_variation_detected)
			(solar_panel_fault)
			(environmental_hazard_detected)
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