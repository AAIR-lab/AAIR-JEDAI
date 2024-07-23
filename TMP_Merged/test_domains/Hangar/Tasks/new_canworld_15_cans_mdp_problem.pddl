(define (problem p01)

	(:domain canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object4 object5 object6 object3 - object
		object7 object8 object9 object10 object11 object12 object13 object14 - object
		obj_loc1 obj_loc2 obj_loc0 obj_loc3 obj_loc4 obj_loc5 obj_loc6 - location
		obj_loc7 obj_loc8 obj_loc9 obj_loc10 obj_loc11 obj_loc12 obj_loc13 obj_loc14 - location
		table - location
		gp pd - pose
		traj - trajectory
		defaultloc - location
		initpose - pose
		initloc - location
		fetch - robot

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
		(at object7 obj_loc7)
		(at object8 obj_loc8)
		(at object9 obj_loc9)
		(at object10 obj_loc10)
		(at object11 obj_loc11)
		(at object12 obj_loc12)
		(at object13 obj_loc13)
		(at object14 obj_loc13)

	)

	(:goal (and

	    (ingripper object1 gripper)
	    (terminated)

	))

	(:goal-reward 100) (:metric maximize (reward))

)
