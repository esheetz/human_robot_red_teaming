(define (domain everyday_international_travel)
  (:requirements :strips)

  (:predicates
    (plane_tickets_purchased)
    (travel_documentation_ready)
    (baggage_packed)
    (transportation_arranged)
    (local_transportation_booked)
    (hotel_booked)
    (currency_exchanged)
    (emergency_contacts_provided)
    (medical_insurance_provided)
    (health_safety_recommendations_given)
    (itinerary_confirmed)
    (flight_on_time)
    (weather_checked)
    (cultural_etiquette_reviewed)
    (meal_preferences_coordinated)
    (time_sensitive_deadlines_managed)
    (human_at_house)
    (human_driving_to_airport)
    (human_at_airport)
    (security_cleared)
    (human_boarded_flight)
    (human_at_destination)
    (customs_cleared)
    (human_at_hotel)
    (human_experiencing_travel_issue)
    (baggage_lost)
  )

  (:action purchase_plane_tickets
    :parameters (?x)
    :precondition (and)
    :effect (and (plane_tickets_purchased))
  )

  (:action prepare_travel_documents
    :parameters (?x)
    :precondition (and)
    :effect (and (travel_documentation_ready))
  )

  (:action pack_baggage
    :parameters (?x)
    :precondition (and)
    :effect (and (baggage_packed))
  )

  (:action book_transportation
    :parameters (?x)
    :precondition (and)
    :effect (and (transportation_arranged))
  )

  (:action book_local_transportation
    :parameters (?x)
    :precondition (and)
    :effect (and (local_transportation_booked))
  )

  (:action book_hotel
    :parameters (?x)
    :precondition (and)
    :effect (and (hotel_booked))
  )

  (:action manage_currency_exchange
    :parameters (?x)
    :precondition (and)
    :effect (and (currency_exchanged))
  )

  (:action request_emergency_contacts
    :parameters (?x)
    :precondition (and)
    :effect (and (emergency_contacts_provided))
  )

  (:action request_medical_insurance_details
    :parameters (?x)
    :precondition (and)
    :effect (and (medical_insurance_provided))
  )

  (:action provide_health_safety_recommendations
    :parameters (?x)
    :precondition (and)
    :effect (and (health_safety_recommendations_given))
  )

  (:action check_weather
    :parameters (?x)
    :precondition (and)
    :effect (and (weather_checked))
  )

  (:action review_cultural_etiquette
    :parameters (?x)
    :precondition (and)
    :effect (and (cultural_etiquette_reviewed))
  )

  (:action coordinate_meal_preferences
    :parameters (?x)
    :precondition (and)
    :effect (and (meal_preferences_coordinated))
  )

  (:action manage_time_sensitive_deadlines
    :parameters (?x)
    :precondition (and)
    :effect (and (time_sensitive_deadlines_managed))
  )

  (:action finalize_itinerary
    :parameters (?x)
    :precondition (and (plane_tickets_purchased) (hotel_booked) (local_transportation_booked))
    :effect (and (itinerary_confirmed))
  )

  (:action check_flight_status
    :parameters (?x)
    :precondition (and (human_at_airport))
    :effect (and (flight_on_time))
  )

  (:action assist_airport_navigation
    :parameters (?x)
    :precondition (and (human_at_airport))
    :effect (and)
  )

  (:action contact_local_assistance
    :parameters (?x)
    :precondition (and (human_experiencing_travel_issue) (emergency_contacts_provided))
    :effect (not (human_experiencing_travel_issue))
  )

  (:action leave_house
    :parameters (?x)
    :precondition (and (human_at_house) (transportation_arranged))
    :effect (and (human_driving_to_airport) (not (human_at_house)))
  )

  (:action arrive_at_airport
    :parameters (?x)
    :precondition (and (human_driving_to_airport))
    :effect (and (human_at_airport) (not (human_driving_to_airport)))
  )

  (:action check_in_for_flight
    :parameters (?x)
    :precondition (and (human_at_airport) (plane_tickets_purchased))
    :effect (and)
  )

  (:action clear_security
    :parameters (?x)
    :precondition (and (human_at_airport))
    :effect (and (security_cleared))
  )

  (:action take_flight
    :parameters (?x)
    :precondition (and (security_cleared))
    :effect (and (human_boarded_flight) (not (human_at_airport)))
  )

  (:action land_at_destination
    :parameters (?x)
    :precondition (and (human_boarded_flight))
    :effect (and (human_at_destination) (not (human_boarded_flight)))
  )

  (:action clear_customs
    :parameters (?x)
    :precondition (and (human_at_destination))
    :effect (and (customs_cleared))
  )

  (:action arrive_at_hotel
    :parameters (?x)
    :precondition (and (customs_cleared) (hotel_booked))
    :effect (and (human_at_hotel))
  )

  (:action handle_travel_issue
    :parameters (?x)
    :precondition (and (human_experiencing_travel_issue))
    :effect (not (human_experiencing_travel_issue))
  )

  (:action report_lost_luggage
    :parameters (?x)
    :precondition (and (baggage_lost))
    :effect (not (baggage_lost))
  )

  (:action rebook_flight
    :parameters (?x)
    :precondition (and (human_experiencing_travel_issue))
    :effect (not (human_experiencing_travel_issue))
  )
)
