(define (problem p01)

	(:domain DelicateCanDeterministic)
	(:objects

		gripper - manip
		black_can pink_can purple_can brown_can cream_can maroon_can green_can - can
		blue_can grey_can red_can - can
		traj - trajectory
		fetch - robot

	)

	(:init
		(empty gripper)
		(delicate brown_can)
		(delicate maroon_can)
		(delicate red_can)

	)

	(:goal (and

	    (ingripper black_can gripper)
	))

)