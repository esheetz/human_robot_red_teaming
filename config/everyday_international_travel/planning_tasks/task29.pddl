(define (problem task29)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(not (currency_exchanged))
			(baggage_lost)
			(not (jet_lag_recommendations_given))
			(human_mugged)
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)