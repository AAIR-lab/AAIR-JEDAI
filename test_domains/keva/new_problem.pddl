(define (problem keva)
(:domain KevaDeterministic)
(:objects
  plank1 plank2 plank3 plank4 plank5 plank6 plank7 plank8 plank9 plank10 - plank
  yumi - robot
  left right - gripper
)



(:init
  (empty_gripper left)
  (empty_gripper right)
  (can_reach left plank1)
  (can_reach left plank2)
  (can_reach left plank3)
  (can_reach left plank4)
  (can_reach left plank5)
  (can_reach right plank6)
  (can_reach right plank7)
  (can_reach right plank8)
  (can_reach right plank9)
  (can_reach right plank10)
  (onTable plank1)
  (onTable plank2)
  (onTable plank3)
  (onTable plank4)
  (onTable plank5)
  (onTable plank6)
  (onTable plank7)
  (onTable plank8)
  (onTable plank9)
  (onTable plank10)
  (clearPlank plank1)
  (clearPlank plank2)
  (clearPlank plank3)
  (clearPlank plank4)
  (clearPlank plank5)
  (clearPlank plank6)
  (clearPlank plank7)
  (clearPlank plank8)
  (clearPlank plank9)
  (clearPlank plank10)
)

(:goal (and
    (inGripper left plank1)
)))
