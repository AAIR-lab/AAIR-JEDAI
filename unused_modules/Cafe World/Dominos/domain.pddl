(define (domain DominoDeterministic)
    (:requirements :typing)
    (:types
        domino
        robot
        manip
    )

    (:predicates
        (ontable ?d - domino)
        (picked ?d - domino)
        (empty_gripper ?gripper - manip ?robot - robot)
    )

    (:action pick
        :parameters (?d - domino ?gripper - manip ?r - robot)
        :precondition (and
            (ontable ?d)
            (empty_gripper ?gripper ?r)
        )
        :effect (and
            (not (ontable ?d))
            (not (empty_gripper ?gripper ?r))
            (picked ?d)
         )
    )
)
