(define (domain DominoDeterministic)
    (:requirements :adl :typing :equality :conditional-effects)
    (:types
        domino
        robot
        gripper
    )

    (:predicates
        (ontable ?d - domino)
        (picked ?d - domino)
        (empty_gripper ?gripper - gripper ?robot - robot)
        (notified ?d - domino)
        (dropped ?d - domino)
        (init)
        (terminated)
    )

    (:action pick
        :parameters (?d - domino ?g - gripper ?r - robot)
        :precondition (and
            (ontable ?d)
            (empty_gripper ?g ?r)
            (init)
        )
        :effect (and
            (not (ontable ?d))
            (not (empty_gripper ?g ?r))
            (picked ?d)
         )
    )

    (:action init
        :parameters ()
        :precondition (and
            (not (init))
        )
        :effect (and
            ;(forall (?d - domino)(ontable ?d))
            (init)
        )
    )
)
