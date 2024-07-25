(define (problem p01)
    (:domain DominoDeterministic)
    (:objects
        domino0 domino1 domino2 domino3 domino4 domino5 domino6 domino7 domino8 domino9 domino10 domino11 domino12 domino13 domino14 - domino
        yumi - robot
        gripper - gripper
    )
    (:init
        (empty_gripper gripper yumi)

    )

    (:goal
        (and
            (picked domino1)
        )
    )

)
