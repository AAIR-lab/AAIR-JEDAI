(define (domain canworld)

	(:requirements :typing :strips :adl :equality)

	(:types
		object
		manip
		location
		pose
		trajectory
		color
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
		(object_color ?o - object ?c - color)
		(location_color ?l - location ?c - color)
		(sorted ?o - object)
	)

	(:action grasp
		:parameters(?obj - object ?pose1 ?pose2 - pose ?gripper - manip ?traj - trajectory ?loc - location)
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
                0.8 (and
                        (not (at ?obj ?loc))
                        (robotpose ?pose2)
                        (not (empty ?gripper))
                        (ingripper ?obj ?gripper)
                    )
		    )
	)

	(:action put
		:parameters(?obj - object ?pose1 ?pose2 - pose ?gripper - manip ?traj - trajectory ?tloc - location)
		:precondition (and
			(ingripper ?obj ?gripper)
			(robotpose ?pose1)
			(ispd ?pose2 ?obj ?gripper)
			(forall (?traj - trajectory)(not (putdownobstructs ?traj ?obj)))
			(clear ?tloc)
			(exists (?c - color)(and (object_color ?obj ?c)(location_color ?tloc ?c)))
		)

		:effect(and
			(not (ingripper ?obj ?gripper))
			(robotpose ?pose2)
			(empty ?gripper)
			(at ?obj ?tloc)
			(forall (?o - object) (forall (?t - trajectory) (not (obstructs ?t ?obj ?o))))
			(sorted ?obj)

		)

	)

	(:action done

        :precondition (and (init)
           (forall (?o - object)(sorted ?o))
        )
        :effect (terminated)

    )

    (:action initialize

        :precondition (not (init))
        :effect (and
            (init)

            (isgp gp_ob1 object1 gripper)
            (isgp gp_ob2 object2 gripper)
            (isgp gp_ob0 object0 gripper)

            (ispd pd_ob1 object1 gripper)
            (ispd pd_ob2 object2 gripper)
            (ispd pd_ob0 object0 gripper)

            (ismp traj initpose gp_ob1)
            (ismp traj initpose gp_ob2)
            (ismp traj initpose gp_ob0)

            (clear table1)
            (clear table2)
            (clear table3)
            (color object1 red)
            (color object2 red)
            (color object3 blue)
            (color object4 blue)
            (color table1 white)
            (color table2 red)
            (color table3 blue)

        )

    )

)