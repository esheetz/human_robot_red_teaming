(define (problem task05)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(not (currency_exchanged))
			(not (medical_requirements_validated))
			(human_lost)
			(flight_delayed)
			(not (jet_lag_recommendations_given))
			(not (local_transportation_booked))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)