(define (problem p01)

	(:domain cafeWorld)
	(:objects

		gripper - manipulator
		can_red can_blue can_green can_brown - can
		counter -  location
		table_red table_blue table_green table_brown - location
		starting_point - location
		fetch - robot

	)

	(:init
		(at starting_point fetch)
		(empty gripper)
		(order can_red counter)
		(order can_blue counter)
		(order can_green counter)
		(order can_brown counter)

	)

	(:goal (and

	    (ingripper can_red gripper)
	))
)
