(define (problem p01)

	(:domain canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object4 object5 object6 object3 - object
		object7 object8 object9 object10 object11 object12 object13 object14 - object
		object15 object16 object17 object18 object19 - object
		object20 object21 object22 object23 object24 - object
		obj_loc1 obj_loc2 obj_loc0 obj_loc3 obj_loc4 obj_loc5 obj_loc6 - location
		obj_loc7 obj_loc8 obj_loc9 obj_loc10 obj_loc11 obj_loc12 obj_loc13 obj_loc14 - location
		obj_loc15 obj_loc16 obj_loc17 obj_loc18 obj_loc19 - location
		obj_loc20 obj_loc21 obj_loc22 obj_loc23 obj_loc24 - location
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
		(at object15 obj_loc15)
		(at object16 obj_loc16)
		(at object17 obj_loc17)
		(at object18 obj_loc18)
		(at object19 obj_loc19)
		(at object20 obj_loc20)
		(at object21 obj_loc21)
		(at object22 obj_loc22)
		(at object23 obj_loc23)
		(at object24 obj_loc24)
	)

	(:goal (and

	    (at object1 table)
	    (terminated)

	))

	(:goal-reward 100) (:metric maximize (reward))

)
