(define (problem task50)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(human_lost)
			(not (medical_requirements_validated))
			(not (valid_visa))
			(not (local_transportation_booked))
			(flight_delayed)
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)