(define (domain KevaDeterministic)
  (:requirements :strips :typing)

  (:types robot plank gripper)

  (:predicates
    (onTable ?p - plank)
    (onSinglePlank ?p1 - plank ?p2 - plank)
    (onDoublePlank ?p1 - plank ?p2 - plank ?p3 - plank)
    (clearPlank ?p - plank)
    (empty_gripper ?gripper - gripper )
    (inGripper ?gripper - gripper ?p - plank)
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
      (inGripper  ?gripper ?plank)
      (not (clearPlank ?plank))
      (not (onTable ?plank))
    )
  ) 

  (:action putDown_plank_vertical_onTable
    :parameters (?gripper - gripper ?p - plank ?rob - robot)
    :precondition (and
      (inGripper ?gripper ?p)
      (not (placed ?p)))
    :effect (and
      (empty_gripper ?gripper)
      (onTable ?p)
      (clearPlank ?p)
      (vertical ?p)
      (not (inGripper ?gripper ?p))
      (placed ?p))
  )



  (:action putDown_plank_vertical_onPlank
    :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?rob - robot)
    :precondition (and
      (inGripper  ?gripper ?p1)
      (not (= ?p1 ?p2))
      (not (placed ?p1))
      (placed ?p2))
    :effect (and
      (empty_gripper ?gripper)
      (onSinglePlank ?p1 ?p2)
      (not (clearPlank ?p2))
      (vertical ?p1)
      (not (inGripper  ?gripper ?p1))
      (clearPlank ?p1)
      (placed ?p1))
  )




  (:action putDown_plank_horizontal_onDoublePlank_both_vertical
    :parameters (?gripper - gripper ?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      
      (inGripper ?gripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (vertical ?p2)
      (vertical ?p3))
    :effect (and (empty_gripper ?gripper) 
      (not (inGripper ?gripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (horizontal ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )













)
