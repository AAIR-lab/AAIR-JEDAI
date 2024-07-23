(define (problem canworld1) (:domain robotics)
(:objects
object0 object1 object2 - object

loc_object0 loc_object1 loc_object2 arm_gp_object0 arm_gp_object1 arm_gp_object2 - location

robotInitLoc - location
table6 - location

arm_pdp_object0_table6 arm_pdp_object1_table6 arm_pdp_object2_table6 - location


gripper - manip

)

(:init
(At object0 loc_object0)
(At object1 loc_object1)
(At object2 loc_object2)

(RobotAt robotInitLoc)

(empty gripper)

(IsGP arm_gp_object0 object0 gripper)
(IsGP arm_gp_object1 object1 gripper)
(IsGP arm_gp_object2 object2 gripper)

(IsAccessPointFor arm_pdp_object0_table6 object0 table6 gripper)
(IsAccessPointFor arm_pdp_object1_table6 object1  table6 gripper)
(IsAccessPointFor arm_pdp_object2_table6 object2 table6 gripper)

(clear table6)
)

(:goal (and (not (At object1 loc_object1))))
)