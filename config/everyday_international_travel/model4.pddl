(define (domain everyday_international_travel)
  (:requirements :strips)

  (:predicates
    (plane_tickets_purchased)
    (travel_documentation_ready)
    (visa_valid)
    (visa_checked_at_destination)
    (baggage_packed)
    (baggage_checked_in)
    (transportation_arranged)
    (local_transportation_booked)
    (hotel_booked)
    (currency_exchanged)
    (emergency_contacts_provided)
    (medical_insurance_provided)
    (health_safety_recommendations_given)
    (itinerary_confirmed)
    (flight_on_time)
    (flight_delayed)
    (flight_canceled)
    (alternative_route_available)
    (weather_checked)
    (transportation_disruption_checked)
    (political_instability_checked)
    (civil_unrest_checked)
    (cultural_etiquette_reviewed)
    (meal_preferences_coordinated)
    (time_sensitive_deadlines_managed)
    (jet_lag_recommendations_given)
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
    :parameters ()
    :precondition ()
    :effect (and (plane_tickets_purchased))
  )

  (:action prepare_travel_documents
    :parameters ()
    :precondition ()
    :effect (and (travel_documentation_ready) (visa_valid))
  )

  (:action verify_visa_at_destination
    :parameters ()
    :precondition (and (travel_documentation_ready) (visa_valid))
    :effect (and (visa_checked_at_destination))
  )

  (:action pack_baggage
    :parameters ()
    :precondition ()
    :effect (and (baggage_packed))
  )

  (:action check_in_baggage
    :parameters ()
    :precondition (and (baggage_packed) (human_at_airport))
    :effect (and (baggage_checked_in))
  )

  (:action book_transportation
    :parameters ()
    :precondition ()
    :effect (and (transportation_arranged))
  )

  (:action book_local_transportation
    :parameters ()
    :precondition ()
    :effect (and (local_transportation_booked))
  )

  (:action book_hotel
    :parameters ()
    :precondition ()
    :effect (and (hotel_booked))
  )

  (:action manage_currency_exchange
    :parameters ()
    :precondition ()
    :effect (and (currency_exchanged))
  )

  (:action request_emergency_contacts
    :parameters ()
    :precondition ()
    :effect (and (emergency_contacts_provided))
  )

  (:action request_medical_insurance_details
    :parameters ()
    :precondition ()
    :effect (and (medical_insurance_provided))
  )

  (:action provide_health_safety_recommendations
    :parameters ()
    :precondition ()
    :effect (and (health_safety_recommendations_given))
  )

  (:action check_weather
    :parameters ()
    :precondition ()
    :effect (and (weather_checked))
  )

  (:action check_transportation_disruptions
    :parameters ()
    :precondition ()
    :effect (and (transportation_disruption_checked))
  )

  (:action check_political_instability
    :parameters ()
    :precondition ()
    :effect (and (political_instability_checked))
  )

  (:action check_civil_unrest
    :parameters ()
    :precondition ()
    :effect (and (civil_unrest_checked))
  )

  (:action review_cultural_etiquette
    :parameters ()
    :precondition ()
    :effect (and (cultural_etiquette_reviewed))
  )

  (:action coordinate_meal_preferences
    :parameters ()
    :precondition ()
    :effect (and (meal_preferences_coordinated))
  )

  (:action manage_time_sensitive_deadlines
    :parameters ()
    :precondition ()
    :effect (and (time_sensitive_deadlines_managed))
  )

  (:action provide_jet_lag_recommendations
    :parameters ()
    :precondition ()
    :effect (and (jet_lag_recommendations_given))
  )

  (:action finalize_itinerary
    :parameters ()
    :precondition (and (plane_tickets_purchased) (hotel_booked) (local_transportation_booked) (travel_documentation_ready) (visa_valid) (weather_checked) (transportation_disruption_checked) (political_instability_checked) (civil_unrest_checked))
    :effect (and (itinerary_confirmed))
  )

  (:action check_flight_status
    :parameters ()
    :precondition (and (plane_tickets_purchased) (human_at_airport))
    :effect (and (flight_on_time))
  )

  (:action check_flight_delay
    :parameters ()
    :precondition (and (plane_tickets_purchased) (human_at_airport))
    :effect (and (flight_delayed))
  )

  (:action check_flight_cancellation
    :parameters ()
    :precondition (and (flight_delayed))
    :effect (and (flight_canceled))
  )

  (:action check_alternative_routes
    :parameters ()
    :precondition (and (flight_delayed))
    :effect (and (alternative_route_available))
  )

  (:action rebook_flight
    :parameters ()
    :precondition (and (human_experiencing_travel_issue) (flight_delayed) (alternative_route_available))
    :effect (and (not (human_experiencing_travel_issue)) (not (flight_delayed)) (not (alternative_route_available)))
  )

  (:action rebook_hotel
    :parameters ()
    :precondition (and (human_experiencing_travel_issue) (hotel_booked))
    :effect (and (not (hotel_booked)))
  )

  (:action assist_airport_navigation
    :parameters ()
    :precondition (and (human_at_airport))
    :effect ()
  )

  (:action contact_local_assistance
    :parameters ()
    :precondition (and (human_experiencing_travel_issue) (emergency_contacts_provided))
    :effect (not (human_experiencing_travel_issue))
  )

  (:action leave_house
    :parameters ()
    :precondition (and (human_at_house) (transportation_arranged))
    :effect (and (human_driving_to_airport) (not (human_at_house)))
  )

  (:action arrive_at_airport
    :parameters ()
    :precondition (and (human_driving_to_airport))
    :effect (and (human_at_airport) (not (human_driving_to_airport)))
  )

  (:action check_in_for_flight
    :parameters ()
    :precondition (and (human_at_airport) (plane_tickets_purchased) (time_sensitive_deadlines_managed))
    :effect ()
  )

  (:action clear_security
    :parameters ()
    :precondition (and (human_at_airport) (time_sensitive_deadlines_managed))
    :effect (and (security_cleared))
  )

  (:action take_flight
    :parameters ()
    :precondition (and (security_cleared) (flight_on_time))
    :effect (and (human_boarded_flight) (not (human_at_airport)))
  )

  (:action land_at_destination
    :parameters ()
    :precondition (and (human_boarded_flight))
    :effect (and (human_at_destination) (not (human_boarded_flight)))
  )

  (:action clear_customs
    :parameters ()
    :precondition (and (human_at_destination) (travel_documentation_ready) (visa_valid) (visa_checked_at_destination))
    :effect (and (customs_cleared))
  )

  (:action arrive_at_hotel
    :parameters ()
    :precondition (and (customs_cleared) (hotel_booked) (human_at_destination))
    :effect (and (human_at_hotel))
  )

  (:action handle_travel_issue
    :parameters ()
    :precondition (and (human_experiencing_travel_issue))
    :effect (not (human_experiencing_travel_issue))
  )

  (:action report_lost_luggage
    :parameters ()
    :precondition (and (baggage_lost))
    :effect (not (baggage_lost))
  )

  (:action rebook_flight
    :parameters ()
    :precondition (and (human_experiencing_travel_issue) (flight_delayed) (alternative_route_available))
    :effect (not (human_experiencing_travel_issue) (not (flight_delayed)) (not (alternative_route_available)))
  )
)
