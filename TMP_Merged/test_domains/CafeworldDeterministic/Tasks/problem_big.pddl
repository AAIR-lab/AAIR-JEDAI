(define (problem p01)

	(:domain cafeWorld)
	(:objects

		gripper - manipulator
		canred canblue cangreen canbrown - can
		counter tablered tableblue tablegreen tablebrown  - location
		fetch - robot

	)

	(:init
		(empty gripper)

        (teleported tablered fetch)
        (teleported tableblue fetch)
        (teleported tablegreen fetch)
        (teleported tablebrown fetch)

		(order canred counter)
		(order canblue counter)
		(order cangreen counter)
		(order canbrown counter)
	)

	(:goal (and

		(at counter fetch)

	))
)
