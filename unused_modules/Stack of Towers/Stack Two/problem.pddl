(define (problem hanoi3)
  (:domain HanoiDeterministic)
  
  (:objects 
      location_i location_ii location_iii - loc
      box_small box_large - box
      fetch - robot
  )
  (:init
   (clear location_iii)
   (clear box_small)
   (clear box_large)
   (on box_large location_i)
   (on box_small location_ii)
  )

  (:goal (and
    (on box_large location_iii)
    (on box_small box_large)
    )
  )
)