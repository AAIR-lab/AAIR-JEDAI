(define (problem keva)
(:domain KevaDeterministic)
(:objects
  plank1 plank2 plank3 - plank
  location1 location2 location3 - location
  yumi - robot
)



(:init
  (handempty)
  (free location2)
  (human_placed location1 plank1)
  (human_placed location2 plank2)
  (placed location3 plank3)
  (inBuffer plank3)
  (clearPlank plank1)
  (clearPlank plank2)
  (clearPlank plank3)
)

(:goal (and
  (onTable plank1)
	(vertical plank1)
	(onTable plank2)
	(vertical plank2)
	(onDoublePlank plank3 plank1 plank2)
	(horizontal plank3)
)))
