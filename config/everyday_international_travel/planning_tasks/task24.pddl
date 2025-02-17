(define (problem task24)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(not (local_transportation_booked))
			(not (medical_requirements_validated))
			(flight_delayed)
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)