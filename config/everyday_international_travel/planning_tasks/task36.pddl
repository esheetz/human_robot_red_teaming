(define (problem task36)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(not (medical_requirements_validated))
			(not (currency_exchanged))
			(not (local_transportation_booked))
			(flight_delayed)
			(human_mugged)
			(not (jet_lag_recommendations_given))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)