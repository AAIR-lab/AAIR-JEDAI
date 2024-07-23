(define (problem canworld1) (:domain robotics)
(:objects
object0 object1 object2 object3 object4 object5 object6 object7 object8 object9 object10 object11 object12 object13 - object

loc_object0 loc_object1 loc_object2 loc_object3 loc_object4 loc_object5 loc_object6 loc_object7 loc_object8 loc_object9 loc_object10 loc_object11 loc_object12 loc_object13 loc_object14 arm_gp_object0 arm_gp_object1 arm_gp_object2 arm_gp_object3 arm_gp_object4 arm_gp_object5 arm_gp_object6 arm_gp_object7 arm_gp_object8 arm_gp_object9 arm_gp_object10 arm_gp_object11 arm_gp_object12 arm_gp_object13 - location

robotInitLoc - location
table6 - location

arm_pdp_object0_table6 arm_pdp_object1_table6 arm_pdp_object2_table6 arm_pdp_object3_table6 arm_pdp_object4_table6 arm_pdp_object5_table6 arm_pdp_object6_table6 arm_pdp_object7_table6 arm_pdp_object8_table6 arm_pdp_object9_table6 arm_pdp_object10_table6 arm_pdp_object11_table6 arm_pdp_object12_table6 arm_pdp_object13_table6 - location

gripper - manip

)

(:init
(at object0 loc_object0)
(at object1 loc_object1)
(at object2 loc_object2)
(at object3 loc_object3)
(at object4 loc_object4)
(at object5 loc_object5)
(at object6 loc_object6)
(at object7 loc_object7)
(at object8 loc_object8)
(at object9 loc_object9)
(at object10 loc_object10)
(at object11 loc_object11)
(at object12 loc_object12)
(at object13 loc_object13)


(robotat robotInitLoc)

(empty gripper)

(isgp arm_gp_object0 object0 gripper)
(isgp arm_gp_object1 object1 gripper)
(isgp arm_gp_object2 object2 gripper)
(isgp arm_gp_object3 object3 gripper)
(isgp arm_gp_object4 object4 gripper)
(isgp arm_gp_object5 object5 gripper)
(isgp arm_gp_object6 object6 gripper)
(isgp arm_gp_object7 object7 gripper)
(isgp arm_gp_object8 object8 gripper)
(isgp arm_gp_object9 object9 gripper)
(isgp arm_gp_object10 object10 gripper)
(isgp arm_gp_object11 object11 gripper)
(isgp arm_gp_object12 object12 gripper)
(isgp arm_gp_object13 object13 gripper)



(isaccesspointfor arm_pdp_object0_table6 object0 table6 gripper)
(isaccesspointfor arm_pdp_object1_table6 object1  table6 gripper)
(isaccesspointfor arm_pdp_object2_table6 object2 table6 gripper)
(isaccesspointfor arm_pdp_object3_table6 object3 table6 gripper)
(isaccesspointfor arm_pdp_object4_table6 object4 table6 gripper)
(isaccesspointfor arm_pdp_object5_table6 object5 table6 gripper)
(isaccesspointfor arm_pdp_object6_table6 object6 table6 gripper)
(isaccesspointfor arm_pdp_object7_table6 object7 table6 gripper)
(isaccesspointfor arm_pdp_object8_table6 object8 table6 gripper)
(isaccesspointfor arm_pdp_object9_table6 object9 table6 gripper)
(isaccesspointfor arm_pdp_object10_table6 object10  table6 gripper)
(isaccesspointfor arm_pdp_object11_table6 object11  table6 gripper)
(isaccesspointfor arm_pdp_object12_table6 object12 table6 gripper)
(isaccesspointfor arm_pdp_object13_table6 object13 table6 gripper)



(clear table6)
)

(:goal (and (not (at object1 loc_object1))))


)