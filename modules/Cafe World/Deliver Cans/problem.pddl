(define (problem p01)

	(:domain cafeWorld)
	(:objects

		gripper - manipulator
		can_red can_blue can_green can_brown - can
		counter table_red table_blue table_green table_brown - location
		fetch - robot

	)

	(:init
		(empty gripper)
		(order can_red counter)
		(order can_blue counter)
		(order can_green counter)
		(order can_brown counter)

	)

	(:goal (and

	    (order can_red table_red)
		(order can_blue table_blue)
		(order can_green table_green)
		(order can_brown table_brown)

	))
)
