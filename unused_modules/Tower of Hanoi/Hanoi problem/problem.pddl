(define (problem hanoi3)
  (:domain HanoiDeterministic)
  
  (:objects 
      location_i location_ii location_iii - loc
      box_small box_medium box_large - box
      fetch - robot
  )
  (:init
   (clear location_ii)
   (clear location_iii)
   (clear box_small)
   (on box_large location_i)
   (on box_medium box_large)
   (on box_small box_medium)
   (too_big_to_place_on box_large box_medium)
   (too_big_to_place_on box_large box_small)
   (too_big_to_place_on box_medium box_small)

   (too_big_to_place_on box_medium box_medium)
   (too_big_to_place_on box_small box_small)
   (too_big_to_place_on box_large box_large)

  )

  (:goal (and
    (on box_large location_iii)
    (on box_medium box_large)
    (on box_small box_medium)
    )
  )
)