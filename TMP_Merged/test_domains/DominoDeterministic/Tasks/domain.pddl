(define (domain domino)
    (:requirements :adl :typing :equality :conditional-effects)
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
)