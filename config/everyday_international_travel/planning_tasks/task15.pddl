(define (problem task15)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(human_mugged)
			(not (local_transportation_booked))
			(not (currency_exchanged))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)