(define (problem pi)
(:domain KevaDeterministic)
(:objects
  plank_i plank_ii plank_iii plank_iv plank_v plank_vi plank_vii plank_viii plank_ix plank_x - plank
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
  (can_reach right plank_vii)
  (can_reach right plank_viii)
  (can_reach right plank_ix)
  (can_reach right plank_x)
  (onTable plank_i)
  (onTable plank_ii)
  (onTable plank_iii)
  (onTable plank_iv)
  (onTable plank_v)
  (onTable plank_vi)
  (onTable plank_vii)
  (onTable plank_viii)
  (onTable plank_ix)
  (onTable plank_x)
  (clearPlank plank_i)
  (clearPlank plank_ii)
  (clearPlank plank_iii)
  (clearPlank plank_iv)
  (clearPlank plank_v)
  (clearPlank plank_vi)
  (clearPlank plank_vii)
  (clearPlank plank_viii)
  (clearPlank plank_ix)
  (clearPlank plank_x)
)

(:goal (and
    (inGripper left plank_i)
)))
