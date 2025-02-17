(define (problem task32)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(human_mugged)
			(flight_delayed)
			(not (valid_visa))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)