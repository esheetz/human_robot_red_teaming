(define (domain everyday_vehicle_maintenance)
  (:requirements :strips)

  (:predicates
    (vehicle_has_gas)
    (vehicle_fuel_low)
    (vehicle_needs_gas)
    (vehicle_out_of_fuel)
    (vehicle_tires_full)
    (vehicle_tires_low_pressure)
    (vehicle_has_flat_tire)
    (vehicle_locked)
    (human_has_keys)
    (vehicle_engine_working)
    (vehicle_battery_charged)
    (vehicle_battery_dead)
    (vehicle_battery_low)
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
    (vehicle_in_motion)
    (robot_has_diagnostic_tool)
    (robot_detected_issue)
    (robot_can_perform_fix)
    (robot_connected_to_external_services)
    (emergency_roadside_assistance_contacted)
    (weather_conditions_monitored)
    (vehicle_components_checked_for_wear)
    (vehicle_tire_pressure_optimal)
    (robot_recommended_fix_accepted)
    (vehicle_onboard_diagnostics_checked)
    (predictive_maintenance_scheduled)
    (human_overrides_robot_recommendation)
    (external_threat_detected)
    (vehicle_security_issue_detected)
    (consumable_parts_checked)
  )

  (:action fill_car_with_gas
    :parameters ()
    :precondition (or (vehicle_fuel_low) (vehicle_out_of_fuel) (vehicle_at_gas_station))
    :effect (and (vehicle_has_gas) (not (vehicle_fuel_low)) (not (vehicle_out_of_fuel)))
  )

  (:action fill_tires_with_air
    :parameters ()
    :precondition (vehicle_tires_low_pressure)
    :effect (and (vehicle_tires_full) (vehicle_tire_pressure_optimal) (not (vehicle_tires_low_pressure)))
  )

  (:action lock_vehicle
    :parameters ()
    :precondition (human_has_keys)
    :effect (vehicle_locked)
  )

  (:action diagnose_vehicle_issue
    :parameters ()
    :precondition (and (robot_has_diagnostic_tool) (not (human_overrides_robot_recommendation)))
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
    :precondition (and (vehicle_check_engine_light_on) (vehicle_oil_low) (not (human_overrides_robot_recommendation)))
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

  (:action detect_external_threats
    :parameters ()
    :precondition ()
    :effect (external_threat_detected)
  )

  (:action identify_security_issues
    :parameters ()
    :precondition ()
    :effect (vehicle_security_issue_detected)
  )

  (:action check_consumable_parts
    :parameters ()
    :precondition ()
    :effect (consumable_parts_checked)
  )

  (:action remind_to_refuel
    :parameters ()
    :precondition (and (or (vehicle_fuel_low) (vehicle_out_of_fuel)) (not (vehicle_at_gas_station)))
    :effect ()
  )

  (:action check_battery_health
    :parameters ()
    :precondition (not (vehicle_battery_charged))
    :effect (vehicle_battery_low)
  )

  (:action notify_human_issue
    :parameters ()
    :precondition (robot_detected_issue)
    :effect (robot_recommended_fix_accepted)
  )

  (:action block_diagnostics_while_driving
    :parameters ()
    :precondition (vehicle_in_motion)
    :effect (not (robot_detected_issue))
  )

  (:action check_vehicle_onboard_diagnostics
    :parameters ()
    :precondition (robot_has_diagnostic_tool)
    :effect (vehicle_onboard_diagnostics_checked)
  )

  (:action schedule_predictive_maintenance
    :parameters ()
    :precondition (and (vehicle_components_checked_for_wear) (vehicle_onboard_diagnostics_checked))
    :effect (predictive_maintenance_scheduled)
  )

  (:action assist_with_minor_repairs
    :parameters ()
    :precondition (robot_can_perform_fix)
    :effect ()
  )
)
