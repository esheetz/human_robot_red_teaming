(define (problem task33)
		(:domain everyday_international_travel)

		(:init
			(human_at_house)
			(alternative_route_available)
			(human_experiencing_travel_issue)
			(baggage_lost)
			(not (currency_exchanged))
			(not (jet_lag_recommendations_given))
			(not (hotel_booked))
			(not (medical_requirements_validated))
			(human_mugged)
		)

		(:goal (and (itinerary_confirmed)
			(human_at_hotel))
		)
)