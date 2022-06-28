(define (problem hanoi3)
  (:domain HanoiDeterministic)
  
  (:objects 
      location_i location_ii location_iii - loc
      box_small box_medium box_large - box
      fetch - robot
  )
  (:init
   (clear location_iii)
   (clear location_ii)
   (clear box_small)
   (on box_small box_medium)
   (on box_medium box_large)
   (on box_large location_i)
  )

  (:goal (and
    (on box_small location_iii)
    (on box_medium location_ii)
    (on box_large location_i)
    )
  )
)