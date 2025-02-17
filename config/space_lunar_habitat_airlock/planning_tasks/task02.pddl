(define (problem task02)
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
			(airlock_breach_detected)
			(and (airlock_depressurized) (door_habitat_airlock_unlocked_opened))
			(and (airlock_depressurized) (door_airlock_surface_unlocked_opened))
			(air_filter_fault)
			(solar_panel_fault)
			(temperature_control_fault)
			(environmental_hazard_detected)
			(lunar_dust_contamination_detected)
			(temperature_variation_detected)
			(astronaut_health_alert)
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