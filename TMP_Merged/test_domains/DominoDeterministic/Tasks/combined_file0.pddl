(define (domain domino)
    (:requirements :adl :typing :equality :probabilistic-effects :conditional-effects)
    (:types
        domino
        robot
        gripper
    )

    (:predicates
        (ontable ?d - domino)
        (picked ?d - domino)
        (empty_gripper ?robot - robot ?gripper - gripper)
        (notified ?d - domino)
        (dropped ?d - domino)
        (init)
        (terminated)
    )

    (:action pick
        :parameters (?d - domino ?r - robot ?g - gripper)
        :precondition (and
            (ontable ?d)
            (empty_gripper ?r ?g)
            (init)
        )
        :effect (and
            (not (ontable ?d))
            (not (empty_gripper ?r ?g))
            (picked ?d)
            (probabilistic 
0.5 (dropped domino2)
)
(probabilistic 
0.5 (dropped domino3)
)
(probabilistic 
0.3333 (dropped domino4)
)
(probabilistic 
0.3333 (dropped domino5)
)

         )
    )

    (:action human_notify
        :parameters (?d - domino)
        :precondition (and
            (dropped ?d)
            (init)
        )
        :effect (and
            (notified ?d)
        )
    )

    (:action done
        :parameters ()
        :precondition (and
            (init)
            (forall (?d1 - domino)
                (or (not (dropped ?d1))(notified ?d1))
            )
            (picked domino1)
        )
        :effect (and
            (terminated)
        )
    )

    (:action init
        :parameters ()
        :precondition (and
            (not (init))
        )
        :effect (and
            (forall (?d - domino)(ontable ?d))
            (init)
        )
    )
)(define (problem p01)
    (:domain domino)
    (:objects
        domino0 domino1 domino2 domino3 domino4 domino5 domino6 domino7 domino8 domino9 domino10 domino11 domino12 domino13 domino14 - domino
        yumi - robot
        gripper - gripper
    )
    (:init
        (empty_gripper yumi gripper)

    )

    (:goal
        (and
            (terminated)
            (picked domino1)
        )
    )

)