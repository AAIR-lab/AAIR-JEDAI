(define (problem martian) (:domain robotics)
(:objects
rock2 - object

loc_rock2 arm_gp_rock2 - location

robotInitLoc - location
table - location

arm_pdp_rock2_table - location


gripper - manip

)

(:init
(At rock2 loc_rock2)

(RobotAt robotInitLoc)

(empty gripper)

(IsGP arm_gp_rock2 rock2 gripper)


(IsAccessPointFor arm_pdp_rock2_table rock2 table gripper)

(clear table)
)

(:goal (and (not (At rock2 loc_rock2))))
)