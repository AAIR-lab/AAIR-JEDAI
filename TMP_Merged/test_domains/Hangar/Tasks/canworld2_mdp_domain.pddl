(define (domain canworld2)

    (:requirements :strips :typing :equality )
    (:types
        manip
        object
        location
        pose
        trajectory
    )

    (:predicates

        (init)
        (robotpose ?p - pose)
        (at ?obj - object ?loc - location)
        (in_gripper ?obj - object ?gripper - manip)
        (on_table ?obj - object)
        (isgp ?gp - pose ?obj - object ?gripper - manip)
        (ispdp ?pdp - pose ?obj - object ?gripper - manip)
        (empty ?gripper - manip)
        (obstructs ?traj - trajectory ?o ?obstructingobj - object)
        (putdownobstructs ?traj - trajectory ?obj - object)
        (clear ?l1 - location)
        (terminated)
        (aroundgrasppose ?p - pose)
    )

    (:action movetograsppose
        :parameters(?o - object ?p1 ?p2 - pose ?traj - trajectory ?gripper - manip)
        :precondition (and
            (init)
            (robotpose ?p1)
            (not (= ?p1 ?p2))
            (not (aroundgrasppose ?p2))
            (forall (?obj - object)(not (obstructs ?traj ?o ?obj)))
            (isgp ?p2 ?o ?gripper)
        )
        :effect (and
            (when (not (aroundgrasppose ?p1))(not (robotpose ?p1)))
            (probabilistic
                0.95 (and (robotpose ?p2)(not (robotpose ?p1)))
                0.05 (robotpose around_grasp_pose)
            )
        )
    )

    (:action movetopdpose
        :parameters(?o - object ?p1 ?p2 - pose ?traj - trajectory ?gripper - manip)
        :precondition(and
            (init)
            (robotpose ?p1)
            (not (= ?p1 ?p2))
            (not (aroundgrasppose ?p2))
            (forall (?o - object)(not (putdownobstructs ?traj ?o)))
            (ispdp ?p2 ?o ?gripper)
        )
        :effect (and
           (not (robotpose ?p1))
           (robotpose ?p2)
        )
    )


    (:action grasp
        :parameters(?o1 - object ?objloc - location ?gp - pose ?gripper - manip)
        :precondition (and
            (init)
            (robotpose ?gp)
            (isgp ?gp ?o1 ?gripper)
            (at ?o1 ?objloc)
            (empty ?gripper)
        )
        :effect (and
            (not (empty ?gripper))
            (robotpose initpose)
            (forall (?traj - trajectory) (forall (?o - object) (not (obstructs ?traj ?o1 ?o))))
            (forall (?traj - trajectory) (not (putdownobstructs ?traj ?o1)))
            (clear ?objloc)
            (in_gripper ?o1 ?gripper)
        )
    )

    (:action put
        :parameters(?o1 - object ?target - location ?pdp - pose ?gripper - manip)
        :precondition(and
            (init)
            (not (empty ?gripper))
            (robotpose ?pdp)
            (ispdp ?pdp ?o1 ?gripper)
            (in_gripper ?o1 ?gripper)
        )
        :effect(and
            (empty ?gripper)
            (not (in_gripper ?o1 ?gripper))
            (at ?o1 ?target)
        )
    )

    (:action done

    :precondition (and (init) (not (at object0 loc_object0)) )
    :effect (terminated)

    )

    (:action initialize

        :precondition (not (init))
        :effect (and

			(init)
            (isgp gp_ob1 object1 gripper)
            (isgp gp_ob2 object2 gripper)
            (isgp gp_ob0 object0 gripper)

            (ispdp pd_ob1 object1 gripper)
            (ispdp pd_ob2 object2 gripper)
            (ispdp pd_ob0 object0 gripper)

            (aroundgrasppose around_grasp_pose)

            (clear table)
        )
    )

)
