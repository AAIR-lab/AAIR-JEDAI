(define (problem p01)

	(:domain cafeWorld)
	(:objects

		gripper - manipulator
		can_red can_blue can_green can_brown - can
		counter - counter_loc 
		table_red table_blue table_green table_brown - table
		starting_point - location
		fetch - robot

	)

	(:init
		(empty gripper)
		(at starting_point fetch)
		(order can_red counter)
		(order can_blue counter)
		(order can_green counter)
		(order can_brown counter)

	)

	(:goal (and

	    (at table_blue fetch)
		(order can_red table_blue)

		
	))
)
