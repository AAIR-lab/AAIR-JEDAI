(define (problem keva)
(:domain KevaDeterministic)
(:objects
  plank1 - plank
  location1 - location
  yumi - robot
)



(:init
  (handempty)
  (free location1)
  (clearPlank plank1)
)

(:goal (and
  (onTable plank1)
  (vertical plank1)
)))
