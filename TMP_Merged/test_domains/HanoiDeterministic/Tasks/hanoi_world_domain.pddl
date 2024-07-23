(define (domain hanoi)
  (:requirements :strips)
  (:types
    object
  )
  (:predicates
    (clear ?x - object)
    (on ?x - object ?y - object)
    (bigger ?x - object ?y - object)
    (robot ?r - object)
  )

  (:action move
    :parameters (?robot ?box ?from ?to)
    :precondition (and
                    (not (bigger ?box ?to))
                    (on ?box ?from)
                    (clear ?box)
                    (clear ?to)
                    (robot ?robot)
    )
    :effect (and
                (clear ?from)
                (on ?box ?to)
                (not (on ?box ?from))
		(not (clear ?to))
    )
  )
)
