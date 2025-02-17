(define (problem task23)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(flight_delayed)
			(baggage_lost)
			(not (medical_requirements_validated))
			(not (hotel_booked))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)