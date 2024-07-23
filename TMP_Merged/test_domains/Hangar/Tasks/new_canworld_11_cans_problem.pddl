(define (problem p01)
(:domain canworld)
(:objects
object0 object1 object2 object3 object4 object5 object6 object7 object8 object9 object10 - object 
obj_loc0 obj_loc1 obj_loc2 obj_loc3 obj_loc4 obj_loc5 obj_loc6 obj_loc7 obj_loc8 obj_loc9 obj_loc10 - location 
table - location 
gp_ob0 gp_ob1 gp_ob2 gp_ob3 gp_ob4 gp_ob5 gp_ob6 gp_ob7 gp_ob8 gp_ob9 gp_ob10 - pose 
pd_ob0 pd_ob1 pd_ob2 pd_ob3 pd_ob4 pd_ob5 pd_ob6 pd_ob7 pd_ob8 pd_ob9 pd_ob10 - pose
defaultloc - location
initpose - pose
traj - trajectory
initloc - location
gripper - manip
)
(:init
(at object0 obj_loc0)
(at object1 obj_loc1)
(at object2 obj_loc2)
(at object3 obj_loc3)
(at object4 obj_loc4)
(at object5 obj_loc5)
(at object6 obj_loc6)
(at object7 obj_loc7)
(at object8 obj_loc8)
(at object9 obj_loc9)
(at object10 obj_loc10)
(isgp gp_ob0 object0 gripper)
(isgp gp_ob1 object1 gripper)
(isgp gp_ob2 object2 gripper)
(isgp gp_ob3 object3 gripper)
(isgp gp_ob4 object4 gripper)
(isgp gp_ob5 object5 gripper)
(isgp gp_ob6 object6 gripper)
(isgp gp_ob7 object7 gripper)
(isgp gp_ob8 object8 gripper)
(isgp gp_ob9 object9 gripper)
(isgp gp_ob10 object10 gripper)
(ispd pd_ob0 object0 gripper)
(ispd pd_ob1 object1 gripper)
(ispd pd_ob2 object2 gripper)
(ispd pd_ob3 object3 gripper)
(ispd pd_ob4 object4 gripper)
(ispd pd_ob5 object5 gripper)
(ispd pd_ob6 object6 gripper)
(ispd pd_ob7 object7 gripper)
(ispd pd_ob8 object8 gripper)
(ispd pd_ob9 object9 gripper)
(ispd pd_ob10 object10 gripper)
(clear table)
(empty gripper)
(robotat initloc)
(robotpose initpose)
)
(:goal (and
(at object1 table)
))
)