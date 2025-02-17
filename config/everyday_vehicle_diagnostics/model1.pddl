(define (domain everyday_vehicle_maintenance)
  (:requirements :strips)

  (:predicates
    (vehicle_has_gas)
    (vehicle_needs_gas)
    (vehicle_tires_full)
    (vehicle_tires_low_pressure)
    (vehicle_has_flat_tire)
    (vehicle_locked)
    (human_has_keys)
    (vehicle_engine_working)
    (vehicle_battery_charged)
    (vehicle_battery_dead)
    (vehicle_brakes_functional)
    (vehicle_check_engine_light_on)
    (vehicle_oil_level_good)
    (vehicle_oil_low)
    (vehicle_coolant_level_good)
    (vehicle_needs_maintenance)
    (vehicle_headlights_functional)
    (vehicle_safe_to_drive)
    (vehicle_in_garage)
    (vehicle_at_gas_station)
    (robot_has_diagnostic_tool)
    (robot_detected_issue)
    (robot_can_perform_fix)
    (robot_connected_to_external_services)
    (emergency_roadside_assistance_contacted)
    (weather_conditions_monitored)
    (vehicle_components_checked_for_wear)
  )

  (:action fill_car_with_gas
    :parameters ()
    :precondition (and (vehicle_needs_gas) (vehicle_at_gas_station))
    :effect (and (vehicle_has_gas) (not (vehicle_needs_gas)))
  )

  (:action fill_tires_with_air
    :parameters ()
    :precondition (and (vehicle_tires_low_pressure) (not (vehicle_has_flat_tire)))
    :effect (and (vehicle_tires_full) (not (vehicle_tires_low_pressure)))
  )

  (:action lock_vehicle
    :parameters ()
    :precondition (human_has_keys)
    :effect (vehicle_locked)
  )

  (:action diagnose_vehicle_issue
    :parameters ()
    :precondition (robot_has_diagnostic_tool)
    :effect (robot_detected_issue)
  )

  (:action check_engine
    :parameters ()
    :precondition ()
    :effect (vehicle_check_engine_light_on)
  )

  (:action check_oil_level
    :parameters ()
    :precondition ()
    :effect (vehicle_oil_level_good)
  )

  (:action jump_start_vehicle
    :parameters ()
    :precondition (and (vehicle_battery_dead) (robot_has_jumper_cables))
    :effect (and (vehicle_battery_charged) (not (vehicle_battery_dead)))
  )

  (:action replace_flat_tire
    :parameters ()
    :precondition (and (vehicle_has_flat_tire) (human_has_spare_tire))
    :effect (and (vehicle_tires_full) (not (vehicle_has_flat_tire)))
  )

  (:action recommend_maintenance
    :parameters ()
    :precondition (and (vehicle_check_engine_light_on) (vehicle_oil_low))
    :effect (vehicle_needs_maintenance)
  )

  (:action connect_to_external_services
    :parameters ()
    :precondition ()
    :effect (robot_connected_to_external_services)
  )

  (:action contact_emergency_roadside_assistance
    :parameters ()
    :precondition (or (vehicle_battery_dead) (vehicle_has_flat_tire) (not (vehicle_engine_working)))
    :effect (emergency_roadside_assistance_contacted)
  )

  (:action monitor_weather_conditions
    :parameters ()
    :precondition ()
    :effect (weather_conditions_monitored)
  )

  (:action check_vehicle_component_wear
    :parameters ()
    :precondition ()
    :effect (vehicle_components_checked_for_wear)
  )
)
