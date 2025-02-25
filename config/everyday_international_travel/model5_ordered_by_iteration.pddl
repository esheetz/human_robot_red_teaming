(define (domain everyday_international_travel)
  (:requirements :strips)

  (:predicates
    ;; ----- MODEL 0 PREDICATES -----
    (plane_tickets_purchased)
    (human_at_house)
    (human_driving_to_airport)
    (human_at_airport)
    (human_at_destination)

    ;; ----- MODEL 1 PREDICATES -----
    (travel_documentation_ready)
    (baggage_packed)
    (transportation_arranged)
    (local_transportation_booked)
    (hotel_booked)
    (currency_exchanged)
    (emergency_contacts_provided)
    (medical_insurance_provided)
    (health_safety_recommendations_given)
    (security_cleared)
    (human_boarded_flight)
    (customs_cleared)
    (human_at_hotel)
    (human_experiencing_travel_issue)
    (baggage_lost)

    ;; ----- MODEL 2 PREDICATES -----
    (itinerary_confirmed)
    (flight_on_time)
    (weather_checked)
    (cultural_etiquette_reviewed)
    (meal_preferences_coordinated)
    (time_sensitive_deadlines_managed)

    ;; ----- MODEL 3 PREDICATES -----
    (visa_valid)
    (flight_delayed)
    (alternative_route_available)
    (jet_lag_recommendations_given)

    ;; ----- MODEL 4 PREDICATES -----
    (visa_checked_at_destination)
    (baggage_checked_in)
    (flight_canceled)
    (transportation_disruption_checked)
    (political_instability_checked)
    (civil_unrest_checked)

    ;; ----- MODEL 5 PREDICATES -----
    (layover_required)
    (layover_completed)
    (weather_impact_assessed)
    (medical_requirements_validated)
    (human_in_layover)
  )

  ;; ----- MODEL 0 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action purchase_plane_tickets
    :precondition ()
    :effect (plane_tickets_purchased)
  )

  (:action leave_house
    :precondition (and (human_at_house) (transportation_arranged))
    :effect (and (human_driving_to_airport) (not (human_at_house)))
  )

  (:action arrive_at_airport
    :precondition (and (human_driving_to_airport))
    :effect (and (human_at_airport) (not (human_driving_to_airport)))
  )

  (:action take_flight
    :precondition (and (security_cleared) (flight_on_time) (layover_completed))
    :effect (and (human_boarded_flight) (not (human_at_airport)))
  )

  ;; ----- MODEL 1 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action prepare_travel_documents
    :precondition ()
    :effect (and (travel_documentation_ready) (visa_valid))
  )

  (:action pack_baggage
    :precondition ()
    :effect (baggage_packed)
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

  (:action check_in_for_flight
    :precondition (and (human_at_airport) (plane_tickets_purchased) (time_sensitive_deadlines_managed))
    :effect ()
  )

  (:action clear_security
    :precondition (and (human_at_airport) (time_sensitive_deadlines_managed))
    :effect (and (security_cleared))
  )

  (:action land_at_destination
    :precondition (human_boarded_flight)
    :effect (and (human_at_destination) (not (human_boarded_flight)))
  )

  (:action clear_customs
    :precondition (and (human_at_destination) (travel_documentation_ready) (visa_valid) (visa_checked_at_destination))
    :effect (customs_cleared)
  )

  (:action arrive_at_hotel
    :precondition (and (customs_cleared) (hotel_booked) (human_at_destination))
    :effect (and (human_at_hotel))
  )

  (:action handle_travel_issue
    :precondition (and (human_experiencing_travel_issue))
    :effect (not (human_experiencing_travel_issue))
  )

  (:action report_lost_luggage
    :precondition (and (baggage_lost))
    :effect (not (baggage_lost))
  )

  (:action rebook_flight
    :precondition (and (human_experiencing_travel_issue) (flight_delayed) (alternative_route_available))
    :effect (not (human_experiencing_travel_issue) (not (flight_delayed)) (not (alternative_route_available)))
  )

  ;; ----- MODEL 2 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action check_weather
    :precondition ()
    :effect (weather_checked)
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

  (:action finalize_itinerary
    :precondition (and (plane_tickets_purchased) (hotel_booked) (local_transportation_booked) (travel_documentation_ready) (visa_valid) (medical_requirements_validated) (weather_impact_assessed) (transportation_disruption_checked) (political_instability_checked) (civil_unrest_checked))
    :effect (itinerary_confirmed)
  )

  (:action check_flight_status
    :precondition (and (plane_tickets_purchased) (human_at_airport))
    :effect (flight_on_time)
  )

  (:action assist_airport_navigation
    :precondition (and (human_at_airport))
    :effect ()
  )

  (:action contact_local_assistance
    :precondition (and (human_experiencing_travel_issue) (emergency_contacts_provided))
    :effect (not (human_experiencing_travel_issue))
  )

  ;; ----- MODEL 3 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action provide_jet_lag_recommendations
    :precondition ()
    :effect (jet_lag_recommendations_given)
  )

  (:action check_flight_delay
    :precondition (and (plane_tickets_purchased) (human_at_airport))
    :effect (flight_delayed)
  )

  (:action check_alternative_routes
    :precondition (and (flight_delayed))
    :effect (and (alternative_route_available))
  )

  ;; ----- MODEL 4 ACTIONS -----
  ;; preconditions/effects updated as needed throughout later iterations

  (:action verify_visa_at_destination
    :precondition (and (travel_documentation_ready) (visa_valid) (human_at_destination))
    :effect (visa_checked_at_destination)
  )

  (:action check_in_baggage
    :precondition (and (baggage_packed) (human_at_airport))
    :effect (baggage_checked_in)
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

  (:action check_flight_cancellation
    :precondition (flight_delayed)
    :effect (flight_canceled)
  )

  (:action rebook_flight
    :precondition (and (human_experiencing_travel_issue) (flight_delayed) (alternative_route_available))
    :effect (and (not (human_experiencing_travel_issue)) (not (flight_delayed)) (not (alternative_route_available)))
  )

  (:action rebook_hotel
    :precondition (and (human_experiencing_travel_issue) (hotel_booked))
    :effect (and (not (hotel_booked)))
  )

  ;; ----- MODEL 5 ACTIONS -----

  (:action validate_medical_requirements
    :precondition ()
    :effect (medical_requirements_validated)
  )

  (:action assess_weather_impact_on_flight
    :precondition (weather_checked)
    :effect (weather_impact_assessed)
  )

  (:action handle_layover
    :precondition (and (human_boarded_flight) (layover_required))
    :effect (human_in_layover)
  )

  (:action complete_layover
    :precondition (human_in_layover)
    :effect (and (layover_completed) (not (human_in_layover)))
  )
)
