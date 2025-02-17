(define (problem task34)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(not (valid_visa))
			(not (hotel_booked))
			(not (currency_exchanged))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)