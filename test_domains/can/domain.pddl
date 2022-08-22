(define (domain DelicateCanDeterministic)
    (:requirements :typing :strips :adl :equality)
    (:types
        object
        manip
        pose
        trajectory
        robot
        base_pose
    )
    (:predicates
        (obstructs ?obstructedobj ?obstructingobj - object ?traj - trajectory)
        (empty ?gripper - manip)
        (ingripper ?gripper - manip ?obj - object)
        (delicate ?obj - object)
        (crushed ?obj - object)
    )
    (:action grasp
        :parameters(?g - manip ?obj - object ?r - robot ?traj - trajectory)
        :precondition(and
            (empty ?g)
            (not (delicate ?obj))
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?g ?obj)
        )
    )

    (:action grasp_delicate
        :parameters(?g - manip ?obj - object ?r - robot ?traj - trajectory)
        :precondition(and
            (empty ?g)
            (delicate ?obj)
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?g ?obj)
            (crushed ?obj)
        )
    )

    (:action put
        :parameters(?g - manip ?obj - object ?r - robot ?traj - trajectory)
        :precondition(and
            (ingripper ?g ?obj)
        )
        :effect(and
            (not (ingripper ?g ?obj))
            (empty ?g)
        )
    )
)
