(define (problem p01)

	(:domain canworld)
	(:objects

		gripper - manip
		object1 object2 object3 object4 object5 - object
		obj_loc1 obj_loc2 obj_loc3 obj_loc4 obj_loc5 - location
		table - location
		gp_ob1 gp_ob2 gp_ob3 gp_ob4 gp_ob5 - pose
		pd_ob1 pd_ob2 pd_ob3 pd_ob4 pd_ob5 - pose
		traj - trajectory
		obj_tloc1 obj_tloc2 obj_tloc3 obj_tloc4 obj_tloc5 - object
		defaultloc - location
		initpose - pose
		initloc - location

	)

	(:init

		(robotpose initpose)
		(robotat initloc)
		(clear table)
		(empty gripper)

		(isgp gp_ob1 object1 gripper)
		(isgp gp_ob2 object2 gripper)
		(isgp gp_ob3 object3 gripper)
		(isgp gp_ob4 object4 gripper)
		(isgp gp_ob5 object5 gripper)


		(ispd pd_ob1 object1 gripper)
		(ispd pd_ob2 object2 gripper)
		(ispd pd_ob3 object3 gripper)
		(ispd pd_ob4 object4 gripper)


		(ismp traj initpose gp_ob1)
		(ismp traj initpose gp_ob2)
		(ismp traj initpose gp_ob3)
		(ismp traj initpose gp_ob4)
		(ismp traj initpose gp_ob5)


		(at object1 obj_loc1)
		(at object2 obj_loc2)
		(at object3 obj_loc3)
		(at object4 obj_loc4)
		(at object5 obj_loc5)


	)

	(:goal (and

		(not (at object1 obj_loc1))

	))

)
