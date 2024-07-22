(define (domain HanoiDeterministic)
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
    :parameters (?box - object ?from - object ?robot - object ?to - object)
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
