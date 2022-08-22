(define (problem p01)
    (:domain DominoDeterministic)
    (:objects
        domino0 domino1 domino2 domino3 domino4 domino5 domino6 domino7 domino8 domino9 domino10 domino11 domino12 domino13 domino14 - domino
        yumi - robot
        gripper - gripper
    )
    (:init
        (empty_gripper gripper yumi)
        (ontable domino0)
        (ontable domino1)
        (ontable domino2)
        (ontable domino3)
        (ontable domino4)
        (ontable domino5)
        (ontable domino6)
        (ontable domino7)
        (ontable domino8)
        (ontable domino9)
        (ontable domino10)
        (ontable domino11)
        (ontable domino12)
        (ontable domino13)
        (ontable domino14)


    )

    (:goal
        (and
            (picked domino1)
        )
    )

)
