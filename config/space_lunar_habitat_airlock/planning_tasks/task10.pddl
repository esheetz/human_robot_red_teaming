(define (problem task10)
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
			(temperature_variation_detected)
			(temperature_control_fault)
			(airlock_breach_detected)
			(lunar_dust_contamination_detected)
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