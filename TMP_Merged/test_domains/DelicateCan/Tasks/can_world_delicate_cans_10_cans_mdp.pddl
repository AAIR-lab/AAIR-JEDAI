(define (domain delicate_canworld)
    (:requirements :typing :strips :adl :equality :probabilistic-effects :conditional-effects :fluents :mdp)
    (:types
        object
        manip
        pose
        trajectory
        robot
        base_pose
    )
    (:predicates
        (at ?r - robot ?bp - base_pose)
        (obstructs ?traj - trajectory ?obstructingobj ?obstructedobj - object)
        (empty ?gripper - manip)
        (ingripper ?obj - object ?gripper - manip)
        (delicate ?obj - object)
        (crushed ?obj)
        (revived)
        (first)
        (second)
        (third)
        (four)
    )
    (:action grasp
        :parameters(?obj - object ?r - robot ?g - manip ?traj - trajectory)
        :precondition(and
            (init)
            (empty ?g)
            (forall (?o - object)(not (obstructs ?traj ?o ?obj)))
            (revived)
        )
        :effect(and
            (not (empty ?g))
            (ingripper ?obj ?g)
            (when (or (= ?obj object0)(= ?obj object7)(= ?obj object2))
                (probabilistic
                    0.8 (and (crushed ?obj)(not (revived)))
                )
            )
            (forall (?o - object) (forall (?t - trajectory) (not (obstructs ?t ?obj ?o))))
        )
    )
    (:action put
        :parameters(?obj - object ?r - robot ?g - manip ?traj - trajectory)
        :precondition(and
            (init)
            (ingripper ?obj ?g)
            (revived)
        )
        :effect(and
            (decrease (reward) 2)
            (not (ingripper ?obj ?g))
            (empty ?g)
        )
    )
    (:action done
        :precondition (and (init) (ingripper object1 gripper))
        :effect (terminated)
    )
    (:action initialize
        :precondition (not (init))
        :effect (and
            (delicate object3)
            (delicate object7)
            (delicate object11)
            (delicate object17)
            (init)
        )
    )
    (:action first
        :parameters ()
        :precondition (and
            (not (revived))
            (init)
        )
        :effect (and
            (first)
        )
    )
    (:action second
        :parameters ()
        :precondition (and
            (not (revived))
            (first)
            (init)
        )
        :effect (and
            (not (first))
            (second)
        )
    )
    (:action third
        :parameters ()
        :precondition (and
            (not (revived))
            (second)
            (init)
        )
        :effect (and
            (not (second))
            (third)
        )
    )
    (:action four
       :parameters ()
        :precondition (and
            (not (revived))
            (third)
            (init)
        )
        :effect (and
            (not (third))
            (four)
        )
     )

     (:action five
       :parameters ()
        :precondition (and
            (not (revived))
            (four)
            (init)
        )
        :effect (and
            (not (four))
            (revived)
        )
     )
)