(define (problem p01)

	(:domain canworld)
	(:objects

		gripper - manip
		object1 object2 object0 - object
		obj_loc1 obj_loc2 obj_loc0 - location
		table - location
		gp_ob1 gp_ob2 gp_ob0 - pose
		pd_ob1 pd_ob2 pd_ob0 - pose
		traj - trajectory
		obj_tloc1 obj_tloc2 obj_tloc0 - location
		defaultloc - location
		initpose - pose
		initloc - location

	)

	(:init

		(robotpose initpose)
		(robotat initloc)
		(clear table)
		(empty gripper)


		(at object1 obj_loc1)
		(at object2 obj_loc2)
		(at object0 obj_loc0)

	)

	(:goal (and

	    (at object1 table)
	    (terminated)

	))

	(:goal-reward 100) (:metric maximize (reward))

)
