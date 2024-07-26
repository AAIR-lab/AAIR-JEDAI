(define (problem p01)

	(:domain delicate_canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object4 object5 object6 object3 - object
		object7 object8 object9 object10 object11 object12 object13 object14 - object
		traj - trajectory
		fetch - robot

	)

	(:init
		(empty gripper)
		(delicate object3)
		(delicate object5)
		(delicate object10)
		(delicate object13)

	)

	(:goal (and

	    (ingripper object1 gripper)
	))
)
