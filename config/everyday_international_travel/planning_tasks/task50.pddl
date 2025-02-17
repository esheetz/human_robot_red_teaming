(define (problem task50)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(human_lost)
			(baggage_lost)
			(flight_delayed)
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)