(define (domain HanoiDeterministic)
  (:requirements :strips :typing)

  (:types
    robot placement_area - object
    box loc - placement_area
  )

  (:predicates
    (clear ?area - placement_area)
    (on ?box - box ?area - placement_area)
  )

  (:action move
    :parameters (?box - box ?from - placement_area ?robot - robot ?to - placement_area)
    :precondition (and
        (on ?box ?from)
        (clear ?box)
        (clear ?to)
    )
    :effect (and
        (clear ?from)
        (on ?box ?to)
        (not (on ?box ?from))
		(not (clear ?to))
    )
  )
)
