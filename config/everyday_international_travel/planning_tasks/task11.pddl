(define (problem task11)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(not (valid_visa))
			(baggage_lost)
			(not (local_transportation_booked))
			(human_mugged)
			(not (jet_lag_recommendations_given))
			(not (currency_exchanged))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)