(define (problem p01)

	(:domain delicate_canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object4 object5 object6 object3 - object
		object7 object8 object9 object10 object11 object12 object13 object14 - object
		object15 object16 object17 object18 object19 - object
		object20 object21 object22 object23 object24 - object
		object25 object26 object27 object28 object29 - object
		traj - trajectory
		fetch - robot

	)

	(:init
		(empty gripper)
		(delicate object3)
		(delicate object5)
		(delicate object10)
		(delicate object13)
		(delicate object17)
		(delicate object23)
		(delicate object27)
	)

	(:goal (and

	    (ingripper object1 gripper)

	))

)
