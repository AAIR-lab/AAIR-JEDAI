(define (problem canworld1) (:domain robotics)
(:objects
object0 object1 - object

loc_object0 loc_object1 arm_gp_object0 arm_gp_object1 - location

robotInitLoc - location
table6 - location

arm_pdp_object0_table6 arm_pdp_object1_table6 - location


gripper - manip

)

(:init
(At object0 loc_object0)
(At object1 loc_object1)

(RobotAt robotInitLoc)

(empty gripper)

(IsGP arm_gp_object0 object0 gripper)
(IsGP arm_gp_object1 object1 gripper)

(IsAccessPointFor arm_pdp_object0_table6 object0 table6 gripper)
(IsAccessPointFor arm_pdp_object1_table6 object1  table6 gripper)
(obstructs arm_gp_object0 object1 object0)

(clear table6)
)

(:goal (and (not (At object0 loc_object0))))
)