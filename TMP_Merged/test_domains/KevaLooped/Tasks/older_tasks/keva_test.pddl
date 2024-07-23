(define (domain keva)
  (:requirements :strips :typing)
  (:types loc plank region)
  (:predicates 
  		   (onTable ?x - loc)
  		   (on ?x - loc ?y - loc)
	       (clearLoc ?x - loc)
	       (handempty)
	       (belongTo ?x - loc ?p - plank)
	       (holding ?p -plank ?x - region)
	       (regionBelongTo ?x - region ?p - plank)
	       )

  (:action pickUp-plank-gripRegion
	     :parameters (?p - plank ?x - region)
	     :precondition (and (handempty) (regionBelongTo ?x ?p))
	     :effect
	     (and 
		   (not (handempty))
		   (holding ?p ?x)
		   ))

  (:action PutDown-plank-gripRegion-side-onTable
		 :parameters (?p - plank ?x - region ?y - loc)
		 :precondition (and (clearLoc ?y) (holding ?p ?x) (belongTo ?y ?p) )
		 :effect
		 (and (not (holding ?p ?x))
		 (not (clearLoc ?y))
		 (handempty)
		 (onTable ?y)))
)
