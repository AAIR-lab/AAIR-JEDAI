(define (problem p01)
(:domain canworld)
(:objects
object0 - object 
obj_loc0 - location 
table - location 
gp_ob0 - pose 
pd_ob0 - pose
defaultloc - location
initpose - pose
traj - trajectory
initloc - location
gripper - manip
)
(:init
(at object0 obj_loc0)
(isgp gp_ob0 object0 gripper)
(ispd pd_ob0 object0 gripper)
(clear table)
(empty gripper)
(robotat initloc)
(robotpose initpose)
)
(:goal (and
(at object0 table)
))
)
