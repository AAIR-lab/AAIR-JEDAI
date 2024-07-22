(define (problem pi)
(:domain KevaDeterministic)
(:objects
  plank_i plank_ii plank_iii plank_iv plank_v plank_vi - plank
  yumi - robot
  left right - gripper
)



(:init
  (empty_gripper left)
  (empty_gripper right)
  (can_reach left plank_i)
  (can_reach left plank_ii)
  (can_reach left plank_iii)
  (can_reach left plank_iv)
  (can_reach left plank_v)
  (can_reach right plank_vi)
  (onTable plank_i)
  (onTable plank_ii)
  (onTable plank_iii)
  (onTable plank_iv)
  (onTable plank_v)
  (onTable plank_vi)
  (clearPlank plank_i)
  (clearPlank plank_ii)
  (clearPlank plank_iii)
  (clearPlank plank_iv)
  (clearPlank plank_v)
  (clearPlank plank_vi)
)

(:goal (and
    (onTable plank_i)
    (vertical plank_i)
    (onTable plank_ii)
    (vertical plank_ii)
    (onDoublePlank plank_iii plank_i plank_ii)
    (horizontal plank_iii)
    (onSinglePlank plank_iv plank_iii)
    (onSinglePlank plank_v plank_iii)
    (onDoublePlank plank_vi plank_iv plank_v)
    (vertical plank_iv)
    (vertical plank_v)
    (horizontal plank_vi)
)))
