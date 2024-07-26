(define (domain delicate_canworld)
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
        (obstructs ?traj - trajectory ?obstructingobj ?obstructedobj - object)
        (empty ?gripper - manip)
        (ingripper ?obj - object ?gripper - manip)
        (delicate ?obj - object)
        (crushed ?obj)
    )
    (:action grasp
        :parameters(?obj - object ?r - robot ?g - manip ?traj - trajectory)
        :precondition(and
            (empty ?g)
            ; (forall (?o - object)(not (obstructs ?traj ?o ?obj)))
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?obj ?g)
            (when (delicate ?obj)
               (crushed ?obj)
            )
            ; (forall (?o - object) (forall (?t - trajectory) (not (obstructs ?t ?obj ?o))))
        )
    )
    (:action put
        :parameters(?obj - object ?r - robot ?g - manip ?traj - trajectory)
        :precondition(and
            (ingripper ?obj ?g)
        )
        :effect(and
            (not (ingripper ?obj ?g))
            (empty ?g)
        )
    )
)