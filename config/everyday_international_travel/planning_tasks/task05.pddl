(define (problem task05)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(human_lost)
			(human_mugged)
			(baggage_lost)
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)