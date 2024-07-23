(define (problem p01)

	(:domain delicate_canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object4 object5 object6 object3 - object
		object7 object8 object9 object10 object11 object12 object13 object14 - object
		object15 object16 object17 object18 object19 - object
		traj - trajectory
		fetch - robot

	)

	(:init
        (revived)
		(empty gripper)

	)

	(:goal (and

	    (ingripper object1 gripper)
	    (terminated)

	))

	(:goal-reward 20) (:metric maximize (reward))

)
