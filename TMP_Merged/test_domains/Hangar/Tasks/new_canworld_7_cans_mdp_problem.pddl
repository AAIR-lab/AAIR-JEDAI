(define (problem p01)

	(:domain canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object4 object 5 object6 - object
		obj_loc1 obj_loc2 obj_loc0 obj_loc3 obj_loc4 obj_loc5 obj_loc6 - location
		table - location
		gp pd - pose
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
		(at object3 obj_loc3)
		(at object4 obj_loc3)
		(at object5 obj_loc3)
		(at object6 obj_loc3)

	)

	(:goal (and

	    (at object1 table)
	    (terminated)

	))

	(:goal-reward 100) (:metric maximize (reward))

)
