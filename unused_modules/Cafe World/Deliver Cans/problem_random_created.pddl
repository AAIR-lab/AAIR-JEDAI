(define (problem p01)	(:domain cafeWorld)	(:objects		gripper - manipulator		can_red can_blue can_green can_brown - can		counter table_red table_blue table_green table_brown init_loc - location		fetch - robot	)	(:init		(at init_loc fetch)		(empty gripper)		(order can_red counter)		(order can_blue counter)		(order can_green counter)		(order can_brown counter)	)	(:goal (and (ingripper can_blue gripper) (at table_blue fetch) (order can_red counter) (order can_brown counter) (order can_green counter) )))