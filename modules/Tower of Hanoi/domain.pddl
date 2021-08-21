(define (domain HanoiDeterministic)
  (:requirements :strips)
  (:types
    box
    location
    Robot
  )
  (:predicates
    (clearlocation ?x - location)
    (clearbox ?x - box)
    (onlocation ?x - box ?y - location)
    (onbox ?x - box ?y - box)
    (bigger ?x - box ?y - box)
  )

  (:action move_from_box_to_location
    :parameters ( ?box - box ?from - box ?robot - Robot ?to - location)
    :precondition (and
                    (onbox ?box ?from)
                    (clearbox ?box)
                    (clearlocation ?to)
    )
    :effect (and
                (clearbox ?from)
                (onlocation ?box ?to)
                (not (onbox ?box ?from))
		            (not (clearlocation ?to))
    )
  )

  (:action move_from_box_to_box
    :parameters ( ?box - box ?from - box ?robot - Robot ?to - box)
    :precondition (and
                    (not (bigger ?box ?to))
                    (onbox ?box ?from)
                    (clearbox ?box)
                    (clearbox ?to)
    )
    :effect (and
                (clearbox ?from)
                (onbox ?box ?to)
                (not (onbox ?box ?from))
		            (not (clearbox ?to))
    )
  )

  (:action move_from_location_to_box
    :parameters ( ?box - box ?from - location ?robot - Robot ?to - box)
    :precondition (and
                    (not (bigger ?box ?to))
                    (onlocation ?box ?from)
                    (clearbox ?box)
                    (clearbox ?to)
    )
    :effect (and
                (clearlocation ?from)
                (onbox ?box ?to)
                (not (onlocation ?box ?from))
		            (not (clearbox ?to))
    )
  )

  (:action move_from_location_to_location
    :parameters ( ?box - box ?from - location ?robot - Robot ?to - location)
    :precondition (and
                    (onlocation ?box ?from)
                    (clearbox ?box)
                    (clearlocation ?to)
    )
    :effect (and
                (clearlocation ?from)
                (onlocation ?box ?to)
                (not (onlocation ?box ?from))
		            (not (clearlocation ?to))
    )
  )

)