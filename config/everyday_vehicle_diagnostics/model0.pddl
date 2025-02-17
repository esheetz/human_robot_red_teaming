(define (domain everyday_vehicle_maintenance)
  (:requirements :strips)

  (:predicates
    (vehicle_has_gas)
    (vehicle_tires_full)
    (vehicle_locked)
    (human_has_keys)
  )

  (:action fill_car_with_gas
    :parameters ()
    :precondition ()
    :effect (vehicle_has_gas)
  )

  (:action fill_tires_with_air
    :parameters ()
    :precondition ()
    :effect (vehicle_tires_full)
  )

  (:action lock_vehicle
    :parameters ()
    :precondition (human_has_keys)
    :effect (vehicle_locked)
  )
)
