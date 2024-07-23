(define (domain keva)
  (:requirements :strips :typing :conditional-effects)
  
  (:types robot plank orientated region location)
  (:predicates
  		   (onTable ?p - plank)
  		   (inBuffer ?p - plank)
  		   (onSinglePlank ?p - plank ?p - plank)
           (onDoublePlank ?p - plank ?p - plank ?p - plank)
           (clearPlank ?p - plank)
  		   (handempty)
             (completed)
  		   (inGripper ?p - plank)
  		   (orientation ?p - plank ?o - orientated)
           (placed ?p - plank)
           (human_placed ?p - plank ?loc - location)
           (free ?loc - location))

  (:constants 
           vertical horizontal  sideways - orientated
           region_1 - region region_2 - region region_3 - region
           location1 - location location2 - location
)

  (:action pickUp_plank_from_region
  		 :parameters (?rob - robot ?p - plank ?r - region ?loc - location)
  		 :precondition (and (handempty) 
                            (clearPlank ?p)
                            (or (human_placed ?p ?loc)(inBuffer ?p)))
  		 :effect (and (not (handempty)) 
                      (inGripper ?p)
                      (not (clearPlank ?p))
                      (not (onTable ?p))
                      (when (human_placed ?p ?loc) (not (human_placed ?p ?loc)))
                      (when (not (free ?loc))(and (free ?loc)))
                      (when (inBuffer ?p)(and (not (inBuffer ?p))))
                      (forall (?pl - plank)
                            (when (and (onSinglePlank ?p ?pl)
                                       (not (= ?p ?pl)))
                                  (and (not(onSinglePlank ?p ?pl))
                                       (clearPlank ?pl)))))
  )



  (:action putDown_plank_vertically_onTable
  		 :parameters (?rob - robot ?p - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p)
                            (not (placed ?p)))
  		 :effect (and (handempty)
                      (onTable ?p)
                      (clearPlank ?p)
                      (orientation ?p vertical)
                      (not (inGripper ?p))
                      (placed ?p))
  )

  (:action putDown_plank_horizontally_onTable
  		 :parameters (?rob - robot ?p - plank)
  		 :precondition (and (not (handempty))
                          (inGripper ?p)
                          (not (placed ?p))
                          (or
                               (orientation ?p vertical)
                               (orientation ?p horizontal)))
  		 :effect (and (handempty)
                    (onTable ?p)
                    (orientation ?p horizontal)
                    (not (inGripper ?p))
                    (clearPlank ?p)
                    (placed ?p))
  )

  (:action putDown_plank_sideways_onTable
  		 :parameters (?rob - robot ?p - plank)
  		 :precondition (and (not (handempty))
                          (inGripper ?p)
                          (not (placed ?p))
                          (or
                               (orientation ?p vertical)
                               (orientation ?p sideways)))
  		 :effect (and (handempty)
                    (onTable ?p)
                    (orientation ?p sideways)
                    (not (inGripper ?p))
                    (clearPlank ?p)
                    (placed ?p))
  )


  (:action putDown_plank_vertically_onPlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank)
  		 :precondition (and (not (handempty))
                          (inGripper ?p1)
                          (not (= ?p1 ?p2))
                          (not (placed ?p1))
                          (placed ?p2))
  		 :effect (and (handempty)
                      (onSinglePlank ?p1 ?p2)
                      (not (clearPlank ?p2))
                      (orientation ?p1 vertical)
                      (not (inGripper ?p1))
                      (clearPlank ?p1)
                      (placed ?p1))
  )

  (:action putDown_plank_horizontally_onSinglePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p1)
                            (not (= ?p1 ?p2))
                            (not (placed ?p1))
                            (placed ?p2))
  		 :effect (and (handempty)
                      (onSinglePlank ?p1 ?p2)
                      (not (inGripper ?p1))
                      (not (clearPlank ?p2))
                      (orientation ?p1 horizontal)
                      (clearPlank ?p1)
                      (placed ?p1))
  )

  (:action putDown_plank_sideways_onSinglePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p1)
                            (not (= ?p1 ?p2))
                            (not (placed ?p1))
                            (placed ?p2))
  		 :effect (and (handempty)
                      (onSinglePlank ?p1 ?p2)
                      (not (inGripper ?p1))
                      (not (clearPlank ?p2))
                      (orientation ?p1 sideways)
                      (clearPlank ?p1)
                      (placed ?p1))
  )

  (:action putDown_plank_horizontally_onDoublePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank ?p3 - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p1)
                            (not (= ?p1 ?p2))
                            (not (= ?p1 ?p3))
                            (not (= ?p2 ?p3))
                            (not (placed ?p1))
                            (placed ?p2)
                            (placed ?p3)
                            (or
                                (and (orientation ?p2 horizontal)
                                     (orientation ?p3 horizontal))
                                (and (orientation ?p2 vertical)
                                     (orientation ?p3 vertical))
                                (and (orientation ?p2 sideways)
                                     (orientation ?p3 sideways))))
  		 :effect (and (handempty)
                      (not (inGripper ?p1))
                      (onDoublePlank ?p1 ?p2 ?p3)
                      (orientation ?p1 horizontal)
                      (not (clearPlank ?p2))
                      (not (clearPlank ?p3))
                      (clearPlank ?p1)
                      (placed ?p1))
  )

  (:action putDown_plank_sideways_onDoublePlank
  		 :parameters (?rob - robot ?p1 - plank ?p2 - plank ?p3 - plank)
  		 :precondition (and (not (handempty))
                            (inGripper ?p1)
                            (not (= ?p1 ?p2))
                            (not (= ?p1 ?p3))
                            (not (= ?p2 ?p3))
                            (not (placed ?p1))
                            (placed ?p2)
                            (placed ?p3)
                            (or
                                (and (orientation ?p2 horizontal)
                                     (orientation ?p3 horizontal))
                                (and (orientation ?p2 sideways)
                                     (orientation ?p3 sideways))
                                (and (orientation ?p2 vertical)
                                     (orientation ?p3 vertical))))
  		 :effect (and (handempty)
                      (not (inGripper ?p1))
                      (onDoublePlank ?p1 ?p2 ?p3)
                      (orientation ?p1 sideways)
                      (not (clearPlank ?p2))
                      (not (clearPlank ?p3))
                      (clearPlank ?p1)
                      (placed ?p1))
  )

  (:action human_place
    :parameters (?p - plank)
    :precondition (and (handempty)
        (free location1)
        (free location2)
    )
    :effect (and
        (orientation ?p sideways)
        (probabilistic
            0.4 (and (human_placed ?p location1)(not (free location1))(clearPlank ?p))
            0.6 (and (human_placed ?p location2)(not (free location2))(clearPlank ?p))
        )
    )
  )

  ; This action is needed to make this problem an SSP for arbitrary
  ; states.
  ;
  ; Example
  ; If the goal itself is to place a plank in a particular location
  ; then the wrong stochastic effect is a deadend.
  ;
  ; This action allows repeated execution until the plank is placed
  ; by the human in the desired location.
  (:action human_remove
    :parameters (?p - plank ?l - location)
    :precondition (and
            (orientation ?p sideways)
            (human_placed ?p ?l)
            (not (free ?l))
            (clearPlank ?p)
    )
    :effect (and
            (not (orientation ?p sideways))
            (not (human_placed ?p ?l))
            (free ?l)
            (not (clearPlank ?p))
    )
  )

  (:action back_to_init
  :parameters (?rob - robot)
  :precondition (and (onTable plank1)
			(orientation plank1 vertical)
			(onTable plank2)
			(orientation plank2 vertical)
			(onDoublePlank plank3 plank1 plank2)
			(orientation plank3 horizontal)
		)

  :effect (and
     (completed)
     )
  )
  
  (:action done
  :precondition ( and (completed))

  :effect (and
     (terminated)
     )
  )
)
