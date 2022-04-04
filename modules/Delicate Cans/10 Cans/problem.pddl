(define (problem p01)

	(:domain DelicateCanDeterministic)
	(:objects

		gripper - manip
		black_can can2 can1 can4 can5 can6 can3 - can
		can7 can8 can9 - can
		traj - trajectory
		fetch - robot

	)

	(:init
		(empty gripper)
		(delicate can3)
		(delicate can5)
		(delicate can9)

	)

	(:goal (and

	    (ingripper black_can gripper)
	))

)