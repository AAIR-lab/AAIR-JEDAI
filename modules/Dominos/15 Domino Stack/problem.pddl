(define (problem p01)
    (:domain DominoDeterministic)
    (:objects
        domino_i domino_ii domino_iii domino_iv domino_v domino_vi domino_vii domino_viii domino_ix domino_x domino_xi domino_xii domino_xiii domino_xiv domino_xv - domino
        yumi - robot
        gripper - manip
    )
    (:init
        (empty_gripper gripper yumi)
        (ontable domino_i)
        (ontable domino_ii)
        (ontable domino_iii)
        (ontable domino_iv)
        (ontable domino_v)
        (ontable domino_vi)
        (ontable domino_vii)
        (ontable domino_viii)
        (ontable domino_ix)
        (ontable domino_x)
        (ontable domino_xi)
        (ontable domino_xii)
        (ontable domino_xiii)
        (ontable domino_xiv)
        (ontable domino_xv)

    )

    (:goal
        (and
            (picked domino_i)
        )
    )
)