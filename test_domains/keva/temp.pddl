(:action putDown_plank_vertical_onTable
    :parameters (?p - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p)
      (not (placed ?p)))
    :effect (and
      (handempty)
      (onTable ?p)
      (clearPlank ?p)
      (vertical ?p)
      (not (inGripper ?p))
      (placed ?p))
  )

  (:action putDown_plank_sideways_onTable
    :parameters (?p - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p)
      (not (placed ?p))
    )
    :effect (and
      (handempty)
      (onTable ?p)
      (clearPlank ?p)
      (sideways ?p)
      (not (inGripper ?p))
      (placed ?p))
  )

  (:action putDown_plank_horizontal_onPlank
    :parameters (?p1 - plank ?p2 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (placed ?p1))
      (placed ?p2))
    :effect (and
      (handempty)
      (onSinglePlank ?p1 ?p2)
      (not (clearPlank ?p2))
      (horizontal ?p1)
      (not (inGripper ?p1))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_vertical_onPlank
    :parameters (?p1 - plank ?p2 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (placed ?p1))
      (placed ?p2))
    :effect (and
      (handempty)
      (onSinglePlank ?p1 ?p2)
      (not (clearPlank ?p2))
      (vertical ?p1)
      (not (inGripper ?p1))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_sideways_onPlank
    :parameters (?p1 - plank ?p2 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (placed ?p1))
      (placed ?p2))
    :effect (and
      (handempty)
      (onSinglePlank ?p1 ?p2)
      (not (clearPlank ?p2))
      (sideways ?p1)
      (not (inGripper ?p1))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_horizontal_onDoublePlank_both_horizontal
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (horizontal ?p2)
      (horizontal ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (horizontal ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_vertical_onDoublePlank_both_horizontal
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (horizontal ?p2)
      (horizontal ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (vertical ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_sideways_onDoublePlank_both_horizontal
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (horizontal ?p2)
      (horizontal ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (sideways ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_horizontal_onDoublePlank_both_vertical
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (vertical ?p2)
      (vertical ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (horizontal ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_vertical_onDoublePlank_both_vertical
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (vertical ?p2)
      (vertical ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (vertical ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_sideways_onDoublePlank_both_vertical
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (vertical ?p2)
      (vertical ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (sideways ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_horizontal_onDoublePlank_both_sideways
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (sideways ?p2)
      (sideways ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (horizontal ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_vertical_onDoublePlank_both_sideways
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (sideways ?p2)
      (sideways ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (vertical ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )

  (:action putDown_plank_sideways_onDoublePlank_both_sideways
    :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?rob - robot)
    :precondition (and
      (not (handempty))
      (inGripper ?p1)
      (not (= ?p1 ?p2))
      (not (= ?p1 ?p3))
      (not (= ?p2 ?p3))
      (not (placed ?p1))
      (placed ?p2)
      (placed ?p3)
      (sideways ?p2)
      (sideways ?p3))
    :effect (and (handempty)
      (not (inGripper ?p1))
      (onDoublePlank ?p1 ?p2 ?p3)
      (sideways ?p1)
      (not (clearPlank ?p2))
      (not (clearPlank ?p3))
      (clearPlank ?p1)
      (placed ?p1))
  )