(define (problem p01)

	(:domain cafeWorld)
	(:objects

		gripper - manipulator
		canred - can
		counter tablered  - location
		fetch - robot

	)

	(:init
		(empty gripper)

        (teleported tablered fetch)

		(order canred counter)
	)

	(:goal (and

	    (at counter fetch)

	))
)
