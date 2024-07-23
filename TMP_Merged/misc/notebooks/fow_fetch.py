# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
import sys
sys.path.append('/home/local/ASUAD/asrinet1/AAIR/TMP_Merged/')
import Config
import pickle
import rospy
rospy.init_node('phys_exec')


# %%
with open(Config.DOMAIN_DIR +'ros_traj.p', 'r') as f:
    trajectories = pickle.load(f)
    f.close()

# %%
sys.path.append(Config.MISC_DIR+"real_world_exec/")
from fetch.fetch_control import Gripper, Head
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import actionlib
from openravepy import *


# %%
gripper = Gripper()


# %%
def execute_manip_traj_on_ros(ros_manip_traj):
    client = actionlib.SimpleActionClient('arm_with_torso_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction)
    print("waiting for server")
    client.wait_for_server(timeout=rospy.Duration(5))
    print("got server")
    id_gen = actionlib.GoalIDGenerator()
    jtg = FollowJointTrajectoryGoal()
    jtg.trajectory = ros_manip_traj
    print("sending goal..")
    client.send_goal(jtg)
    print("waiting")
    client.wait_for_result()
    print("done")   


# %%
def physical_traj_exec(t_type, t_val, robot):
    global gripper
    if robot == 'yumi':
        return
        # if t_type == 'GripperOpenTrajectory':
        #     ret = execute_gripper_open(yumi_ryumi_robotobot)
        # elif t_type == 'GripperCloseTrajectory':
        #     ret = execute_gripper_close()
        # elif t_type == 'ManipTrajectory':
        #     ret = execute_manip(yumi_robot,t_val)
        # elif t_type == 'HandChange':
        #     ret = execute_hand_change() # TODO: implement this
    else:
        if t_type == 'GripperOpenTrajectory':
            gripper.open()
        elif t_type == 'GripperCloseTrajectory':
            gripper.close()
        elif t_type == 'ManipTrajectory':
            execute_manip_traj_on_ros(t_val)

# %% [markdown]
# ## Run trajectories

# %%
def run_trajectory(policy_tree):
    i = 0
    # This function executes the trajectory on openrave and also creates ros trajectories
    root = policy_tree.get_root()
    q = [root]
    while len(q) > 0:
        node = q.pop()
        children = node.get_children()
        if len(children) > 0:
            selected_child = children[0]
            edge = policy_tree.get_edge(node, selected_child)
            if edge.has_mp:
                ll_plan = edge.ll_plan
                exec_sequence = edge.exec_seq
                robot_name = edge.generated_values['robot']
                print(robot_name)
                for arg in exec_sequence:
                    # Execute physical Trajectory here
                    wait = raw_input("Execute Traj {} {} for {} (y/n)".format(str(i), str(ll_plan[arg]['type']), robot_name))
                    if wait == 'n':
                        break
                    print("Executing Traj {} - {}".format(str(i), str(ll_plan[arg]['type'])))
                    val = physical_traj_exec(ll_plan[arg]['type'],ll_plan[arg]['value'], robot_name)
                    i+=1
            q.append(selected_child)
        else:
            break


# %%
run_trajectory(trajectories)

