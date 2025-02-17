(define (problem task12)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(flight_delayed)
			(not (local_transportation_booked))
			(not (valid_visa))
			(not (medical_requirements_validated))
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)