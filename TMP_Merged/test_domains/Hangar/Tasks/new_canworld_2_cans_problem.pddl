(define (problem p01)
(:domain canworld)
(:objects
object0 object1 - object 
obj_loc0 obj_loc1 - location 
table - location 
gp_ob0 gp_ob1 - pose 
pd_ob0 pd_ob1 - pose
defaultloc - location
initpose - pose
traj - trajectory
initloc - location
gripper - manip
fetch - robot
)
(:init
(at object0 obj_loc0)
(at object1 obj_loc1)
(isgp gp_ob0 object0 gripper)
(isgp gp_ob1 object1 gripper)
(ispd pd_ob0 object0 gripper)
(ispd pd_ob1 object1 gripper)
(clear table)
(empty gripper)
(robotat initloc)
(robotpose initpose)
)
(:goal (and
(at object1 table)
))
)
