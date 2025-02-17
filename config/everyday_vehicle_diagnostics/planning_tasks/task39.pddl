(define (problem task39)
		(:domain everyday_vehicle_diagnostics)

		(:init
			(human_has_keys)
			(robot_has_jumper_cables)
			(human_has_spare_tire)
			(not (vehicle_brakes_functional))
			(vehicle_oil_low)
			(vehicle_check_engine_light_on)
			(vehicle_has_flat_tire)
		)

		(:goal (and (vehicle_has_gas)
			(vehicle_tires_full)
			(vehicle_engine_working)
			(vehicle_battery_charged)
			(vehicle_brakes_functional)
			(vehicle_oil_level_good)
			(vehicle_coolant_level_good)
			(vehicle_headlights_functional)
			(vehicle_safe_to_drive)
			(not (vehicle_needs_maintenance)))
		)
)