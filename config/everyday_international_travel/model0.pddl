(define (domain everyday_international_travel)
  (:requirements :strips)

  (:predicates
    (plane_tickets_purchased)
    (human_at_house)
    (human_driving_to_airport)
    (human_at_airport)
    (human_at_destination)
  )

  (:action purchase_plane_tickets
    :precondition ()
    :effect (and (plane_tickets_purchased))
  )

  (:action leave_house
    :precondition (and (human_at_house))
    :effect (and (human_driving_to_airport) (not (human_at_house)))
  )

  (:action arrive_at_airport
    :precondition (and (human_driving_to_airport))
    :effect (and (human_at_airport) (not (human_driving_to_airport)))
  )

  (:action take_flight
    :precondition (and (human_at_airport))
    :effect (and (human_at_destination) (not (human_at_airport)))
  )
)
