(define (problem task43)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(human_lost)
			(not (local_transportation_booked))
			(not (valid_visa))
			(not (jet_lag_recommendations_given))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)