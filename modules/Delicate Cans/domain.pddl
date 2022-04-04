(define (domain DelicateCanDeterministic)
    (:requirements :typing :strips :adl :equality)
    (:types
        can
        manip
        trajectory
        robot
    )
    (:predicates
        (obstructs ?obstructingcan ?obstructedcan - can ?traj - trajectory)
        (empty ?gripper - manip)
        (ingripper ?can - can ?gripper - manip)
        (delicate ?can - can)
        (crushed ?can)
    )
    (:action grasp
        :parameters(?can - can ?g - manip ?r - robot ?traj - trajectory)
        :precondition(and
            (empty ?g)
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?can ?g)

        )
    )
    (:action put
        :parameters(?can - can ?g - manip ?r - robot ?traj - trajectory)
        :precondition(and
            (ingripper ?can ?g)
        )
        :effect(and
            (not (ingripper ?can ?g))
            (empty ?g)
        )
    )
)