(define (problem p01)

	(:domain canworld)
	(:objects

		gripper - manip
		object1 object2 object0 object3- object
		obj_loc1 obj_loc2 obj_loc0 obj_loc3 - location
		table - location
		gp_ob1 gp_ob2 gp_ob0 gp_ob3 - pose
		pd_ob1 pd_ob2 pd_ob0 pd_ob3 - pose
		traj - trajectory
		table1 table2 table3 - location
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

	)

	(:goal (and
	    ;(forall (?o - object)(forall (?l - location) (or (not (at object ?l)) (exist (?c - color) (and (location_color ?l ?c)(object_color ?o ?c))))))
	    (forall (?o - object)(sorted ?o))
	    (terminated)

	))

	(:goal-reward 100) (:metric maximize (reward))

)
