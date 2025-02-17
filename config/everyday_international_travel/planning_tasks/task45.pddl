(define (problem task45)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(baggage_lost)
			(human_mugged)
			(flight_delayed)
			(human_lost)
			(not (local_transportation_booked))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)