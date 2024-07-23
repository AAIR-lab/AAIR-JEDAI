(define (problem canworld1) (:domain robotics)
(:objects
object0 - object

loc_object0 arm_gp_object0 - location

robotInitLoc - location
table6 - location

arm_pdp_object0_table6 - location


gripper - manip

)

(:init
(At object0 loc_object0)

(RobotAt robotInitLoc)

(empty gripper)

(IsGP arm_gp_object0 object0 gripper)

(IsAccessPointFor arm_pdp_object0_table6 object0 table6 gripper)

(clear table6)
)

(:goal (and (not (At object0 loc_object0))))
)