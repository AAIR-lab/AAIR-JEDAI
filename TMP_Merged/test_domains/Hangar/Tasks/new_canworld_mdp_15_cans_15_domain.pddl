(define (domain canworld)

	(:requirements :typing :strips :adl :equality)

	(:types
		object
		manip
		location
		pose
		trajectory
		robot
	)

	(:predicates
		(at ?obj - object ?loc - location)
		(robotat ?loc - location)
		(robotpose ?p - pose)
		(obstructs ?traj - trajectory ?obstructingobj ?obstructedobj - object)
		(putdownobstructs ?traj - trajectory ?obj - object)
		(isgp ?p - pose ?obj - object ?gripper - manip)
		(ispd ?p - pose ?obj - object ?gripper - manip)
		(empty ?gripper - manip)
		(clear ?loc - location)
		(ismp ?traj - trajectory ?p1 ?p2 - pose)
		(ingripper ?obj - object ?gripper - manip)
	)

	(:action grasp
		:parameters(?obj - object ?r - robot ?pose1 ?pose2 - pose ?gripper - manip ?traj - trajectory ?loc - location)
		:precondition(and
			(at ?obj ?loc)
			(robotpose ?pose1)
			(isgp ?pose2 ?obj ?gripper)
			(empty ?gripper)
			;(ismp ?traj ?pose1 ?pose2)
			(forall (?o - object) (not (obstructs ?traj ?o ?obj)))
		)
		:effect
		    (probabilistic
                0.85 (and
                        (not (at ?obj ?loc))
                        (robotpose ?pose2)
                        (not (empty ?gripper))
                        (ingripper ?obj ?gripper)
                    )
		    )
	)

	(:action put
		:parameters(?obj - object ?r - robot ?pose1 ?pose2 - pose ?gripper - manip ?traj - trajectory ?tloc - location)
		:precondition (and
			(ingripper ?obj ?gripper)
			(robotpose ?pose1)
			(ispd ?pose2 ?obj ?gripper)
			(forall (?traj - trajectory)(not (putdownobstructs ?traj ?obj)))
			(clear ?tloc)
		)

		:effect(and
			(not (ingripper ?obj ?gripper))
			(robotpose ?pose2)
			(empty ?gripper)
			(at ?obj ?tloc)
			(forall (?o - object) (forall (?t - trajectory) (not (obstructs ?t ?obj ?o))))

		)

	)

	(:action done

        :precondition (and (init) (ingripper object1 gripper)  )
        :effect (terminated)

    )

    (:action initialize

        :precondition (not (init))
        :effect (and
            (init)

            (isgp gp object1 gripper)
            (isgp gp object2 gripper)
            (isgp gp object0 gripper)
            (isgp gp object3 gripper)
            (isgp gp object5 gripper)
            (isgp gp object4 gripper)
            (isgp gp object6 gripper)
            (isgp gp object7 gripper)
            (isgp gp object8 gripper)
            (isgp gp object9 gripper)
            (isgp gp object10 gripper)
            (isgp gp object11 gripper)
            (isgp gp object12 gripper)
            (isgp gp object13 gripper)
            (isgp gp object14 gripper)

            (ispd pd object1 gripper)
            (ispd pd object2 gripper)
            (ispd pd object0 gripper)
            (ispd pd object3 gripper)
            (ispd pd object4 gripper)
            (ispd pd object5 gripper)
            (ispd pd object6 gripper)
            (ispd pd object7 gripper)
            (ispd pd object8 gripper)
            (ispd pd object9 gripper)
            (ispd pd object10 gripper)
            (ispd pd object11 gripper)
            (ispd pd object12 gripper)
            (ispd pd object13 gripper)
            (ispd pd object14 gripper)

            (clear table)
        )

    )

)