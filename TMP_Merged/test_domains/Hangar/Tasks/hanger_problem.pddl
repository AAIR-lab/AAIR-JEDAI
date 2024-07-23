(define (problem p01)
(:domain hanger)
(:objects 
	camera - sensor
	UAV - agent
	s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 - step
	defaultTraj - trajectory
)	

(:init

    (available UAV)
    (at UAV recharge_station)

    
    (status left_wing fault_exists)
    (status right_wing fault_exists)
)

(:goal (and (fault_detected left_wing)(fault_detected right_wing)(terminated)))
;(:goal (and (at a1 left_wing)(terminated)))
(:goal-reward 100) (:metric maximize (reward))
)
