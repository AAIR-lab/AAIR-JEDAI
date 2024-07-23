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


  (:action PutDown-plank-gripRegion-side-on
		 :parameters (?p1 - plank ?p2 - plank ?p3 - plank ?x - region ?y - loc ?z - loc ?u - loc ?v - loc)
		 :precondition (and (clearLoc ?y) (clearLoc ?z) (clearLoc ?u) (clearLoc ?v) (holding ?p1 ?x) (belongTo ?y ?p1) (belongTo ?u ?p1) (belongTo ?z ?p2) (belongTo ?v ?p3) (not (= ?p1 ?p2)) (not (= ?p2 ?p3)) )
		 :effect
		 (and (not (holding ?p1 ?x))
		 (not (clearLoc ?y))
		 (not (clearLoc ?z))
		 (not (clearLoc ?u))
		 (not (clearLoc ?v))
		 (handempty)
		 (on ?y ?z)
		 (on ?u ?v)))

  

  
)
