(define (domain KevaDeterministic)
  (:requirements :strips :typing)

  (:types robot plank gripper)

  (:predicates
    (onTable ?p - plank)
    (onSinglePlank ?p1 - plank ?p2 - plank)
    (onDoublePlank ?p1 - plank ?p2 - plank ?p3 - plank)
    (clearPlank ?p - plank)
    (empty_gripper ?gripper - gripper )
    (inGripper ?g - gripper ?p - plank)
    (horizontal ?p - plank)
    (vertical ?p - plank)
    (sideways ?p - plank)
    (placed ?p - plank)
    (can_reach ?gripper - gripper ?plank - plank)
  ) 

  (:action pickUp_plank_from_table
    :parameters (?gripper - gripper ?plank - plank ?robot - robot)
    :precondition (and
      (empty_gripper ?gripper)
      (clearPlank ?plank)
      (onTable ?plank)
      (can_reach ?gripper ?plank))
    :effect (and
      (not (empty_gripper ?gripper))
      (inGripper ?gripper ?plank)
      (not (clearPlank ?plank))
      (not (onTable ?plank))
    )
  ) 
)
