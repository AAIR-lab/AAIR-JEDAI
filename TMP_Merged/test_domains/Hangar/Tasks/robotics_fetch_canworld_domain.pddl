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
		:effect(and 
			(not (at ?obj ?loc))
			(robotpose ?pose2)
			(not (empty ?gripper))
			(ingripper ?obj ?gripper)
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

)
