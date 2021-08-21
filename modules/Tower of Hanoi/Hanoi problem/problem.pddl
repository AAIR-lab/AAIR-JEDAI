(define (problem hanoi3)
  (:domain HanoiDeterministic)
  (:objects 
  loc_I loc_II loc_III - location
  box_small box_medium box_large - box
  fetch - Robot
  )
  (:init
   (clearlocation loc_II)
   (clearlocation loc_III)
   (clearbox box_small)
   (onlocation box_large loc_I)
   (onbox box_medium box_large)
   (onbox box_small box_medium)
   (bigger box_large box_medium)
   (bigger box_medium box_small)
  )

  (:goal (and
    (onlocation box_large loc_III)
    (onbox box_medium box_large)
    (onbox box_small box_medium)
    )
  )
)