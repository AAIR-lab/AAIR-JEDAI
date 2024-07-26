(define (problem keva)
(:domain KevaDeterministic)
(:objects plank1 plank2 plank3 - plank
 location1 - location location2 - location
 horizontal vertical sideways - orientated
 yumi - robot
)



(:init (handempty)
       (free location2)
)

(:goal (and (onTable plank1)
			(orientation vertical plank1)
			(onTable plank2)
			(orientation vertical plank2)
			(onDoublePlank plank3 plank1 plank2)
			(orientation horizontal plank3)
			)
))
