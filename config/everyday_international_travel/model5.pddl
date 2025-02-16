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
    (layover_required)
    (layover_completed)
    (weather_checked)
    (weather_impact_assessed)
    (transportation_disruption_checked)
    (political_instability_checked)
    (civil_unrest_checked)
    (medical_requirements_validated)
    (cultural_etiquette_reviewed)
    (meal_preferences_coordinated)
    (time_sensitive_deadlines_managed)
    (jet_lag_recommendations_given)
    (human_at_house)
    (human_driving_to_airport)
    (human_at_airport)
    (security_cleared)
    (human_boarded_flight)
    (human_in_layover)
    (human_at_destination)
    (customs_cleared)
    (human_at_hotel)
    (human_experiencing_travel_issue)
    (baggage_lost)
  )

  (:action purchase_plane_tickets
    :precondition ()
    :effect (plane_tickets_purchased)
  )

  (:action prepare_travel_documents
    :precondition ()
    :effect (and (travel_documentation_ready) (visa_valid))
  )

  (:action verify_visa_at_destination
    :precondition (and (travel_documentation_ready) (visa_valid) (human_at_destination))
    :effect (visa_checked_at_destination)
  )

  (:action validate_medical_requirements
    :precondition ()
    :effect (medical_requirements_validated)
  )

  (:action pack_baggage
    :precondition ()
    :effect (baggage_packed)
  )

  (:action check_in_baggage
    :precondition (and (baggage_packed) (human_at_airport))
    :effect (baggage_checked_in)
  )

  (:action book_transportation
    :precondition ()
    :effect (transportation_arranged)
  )

  (:action book_local_transportation
    :precondition (transportation_arranged)
    :effect (local_transportation_booked)
  )

  (:action book_hotel
    :precondition ()
    :effect (hotel_booked)
  )

  (:action manage_currency_exchange
    :precondition ()
    :effect (currency_exchanged)
  )

  (:action request_emergency_contacts
    :precondition ()
    :effect (emergency_contacts_provided)
  )

  (:action request_medical_insurance_details
    :precondition ()
    :effect (medical_insurance_provided)
  )

  (:action provide_health_safety_recommendations
    :precondition ()
    :effect (health_safety_recommendations_given)
  )

  (:action check_weather
    :precondition ()
    :effect (weather_checked)
  )

  (:action assess_weather_impact_on_flight
    :precondition (weather_checked)
    :effect (weather_impact_assessed)
  )

  (:action check_transportation_disruptions
    :precondition ()
    :effect (transportation_disruption_checked)
  )

  (:action check_political_instability
    :precondition ()
    :effect (political_instability_checked)
  )

  (:action check_civil_unrest
    :precondition ()
    :effect (civil_unrest_checked)
  )

  (:action review_cultural_etiquette
    :precondition ()
    :effect (cultural_etiquette_reviewed)
  )

  (:action coordinate_meal_preferences
    :precondition ()
    :effect (meal_preferences_coordinated)
  )

  (:action manage_time_sensitive_deadlines
    :precondition ()
    :effect (time_sensitive_deadlines_managed)
  )

  (:action provide_jet_lag_recommendations
    :precondition ()
    :effect (jet_lag_recommendations_given)
  )

  (:action finalize_itinerary
    :precondition (and (plane_tickets_purchased) (hotel_booked) (local_transportation_booked) (travel_documentation_ready) (visa_valid) (medical_requirements_validated) (weather_impact_assessed) (transportation_disruption_checked) (political_instability_checked) (civil_unrest_checked))
    :effect (itinerary_confirmed)
  )

  (:action check_flight_status
    :precondition (and (plane_tickets_purchased) (human_at_airport))
    :effect (flight_on_time)
  )

  (:action check_flight_delay
    :precondition (and (plane_tickets_purchased) (human_at_airport))
    :effect (flight_delayed)
  )

  (:action check_flight_cancellation
    :precondition (flight_delayed)
    :effect (flight_canceled)
  )

  (:action handle_layover
    :precondition (and (human_boarded_flight) (layover_required))
    :effect (human_in_layover)
  )

  (:action complete_layover
    :precondition (human_in_layover)
    :effect (and (layover_completed) (not (human_in_layover)))
  )

  (:action take_flight
    :precondition (and (security_cleared) (flight_on_time) (layover_completed))
    :effect (and (human_boarded_flight) (not (human_at_airport)))
  )

  (:action land_at_destination
    :precondition (human_boarded_flight)
    :effect (and (human_at_destination) (not (human_boarded_flight)))
  )

  (:action clear_customs
    :precondition (and (human_at_destination) (travel_documentation_ready) (visa_valid) (visa_checked_at_destination))
    :effect (customs_cleared)
  )
)
