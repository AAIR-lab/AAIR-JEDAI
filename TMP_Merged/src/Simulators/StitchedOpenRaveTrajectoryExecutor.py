import sys
import os
if __name__ == '__main__':
    sys.path.append("/".join(os.path.abspath(os.path.abspath(__file__)).split('/')[:-3]))
    from src.States.OpenRaveLowLevelState import OpenRaveLowLevelState
import ast
import time
import importlib
from src.DataStructures.RefinedPolicy import RefinedPolicy
from src.DataStructures.RefinedPolicyNode import RefinedPolicyNode
from src.DataStructures.RefinedPolicyEdge import RefinedPolicyEdge
import openravepy
import src.util as util
import pickle
from src.Simulators.OpenRaveSimulator import *

def update_run_track_file(run_number):
    # from Config import DOMAIN_DIR
    with open('test_domains/KevaLooped/run_count.txt',"w") as f:
        f.write(str(run_number))
        f.close()

update_run_track_file(1)
import Config


class OpenRaveTrajectoryExecutor:
    def __init__(self,policy_tree=None,ll_state=None):
        if policy_tree is None:
            with open(Config.DOMAIN_DIR + "refined_tree_1.pkl", "rb") as traj_dump:
                self.policy_tree = pickle.load(traj_dump)
                traj_dump.close()
        else:
            self.policy_tree = policy_tree
        if ll_state is None:
            self.ll_state = OpenRaveLowLevelState()
        else:
            self.ll_state = ll_state
        self.sim = self.ll_state.simulator

    @staticmethod
    def select_child_using_props(policy_tree, node, choice, children):
        stochastic_policy = getattr(importlib.import_module('test_domains.'+Config.DOMAIN + '.StochasticPolicy'),'StochasticPolicy')
        return stochastic_policy.select_child_using_props(policy_tree, node, choice, children)


    def convert_to_ros_traj(self, val, speed=0.1):
        # Function to convert openrave trajectories to ros trajectories
        if type(val) == str:
            if '<trajectory>' == val[:12]:
                trajobj = openravepy.RaveCreateTrajectory(self.ll_state.simulator.env, '')
                openravepy.Trajectory.deserialize(trajobj, val)
                desc = trajobj.GetDescription()
                if 'true' in desc:
                    desc = desc.replace('true', 'True')
                if 'false' in desc:
                    desc = desc.replace('false', 'False')
                dct = ast.literal_eval(desc)
                if 'PlanToConfig' in dct['planning_method'] or 'PlanToTSR' in dct['planning_method']:
                    return util.ros_manip_trajectory_from_openrave(self.ll_state.simulator.env.GetRobots()[0], trajobj, velocity_scale=speed)
                elif dct['planning_method'] == 'PlanToBasePose':
                    return util.ros_base_trajectory_from_openrave(self.ll_state.simulator.env.GetRobots()[0], sim, trajobj)
            else:
                return val
        else:
            return val
    
    def run_trajectory(self,num_runs):
        # This function executes the trajectory on openrave and also creates ros trajectories
        ros_traj_list = []
        # choice_seq = [0,1,1,1,0,1,0,1,0,0,1,1,0,1,1,1,0,1,1,0,1,0,1,1]
        choice_seq = [0,1,1,0,1,0,1,0,1,1,0,0,1,1,0,0,0,1,0,1,1,0,1,0]
        # choice_seq = [0,1,0,1,0,1,0,0,1]
        choice_number = 0
        gripper_poses = []
        for i in range(num_runs):
            update_run_track_file(i+1)
            with open(Config.DOMAIN_DIR + "refined_tree_"+str(i+1)+".pkl", "rb") as traj_dump:
                self.policy_tree = pickle.load(traj_dump)
                traj_dump.close()
            root = self.policy_tree.get_root()
            q = [root]
            while len(q) > 0:
                node = q.pop()
                children = node.get_children()
                if len(children) > 0:
                    # Will only enter here for mdp problems
                    if len(children) > 1:
                        # choice = int(raw_input("Choose succes/ failure (1,0): "))
                        choice = choice_seq[choice_number]
                        choice_number+=1
                        ros_traj_list.append(['Placed',choice])
                        #### Only This Portion changes for each domain ###
                        selected_child = OpenRaveTrajectoryExecutor.select_child_using_props(self.policy_tree, node, choice, children)
                        print(selected_child)
                    else:
                        selected_child = children[0]
                    edge = self.policy_tree.get_edge(node, selected_child)
                    if edge.has_mp:
                        ll_plan = edge.ll_plan
                        exec_sequence = edge.exec_seq
                        for arg in exec_sequence:
                            exec_obj = getattr(importlib.import_module('test_domains.'+Config.DOMAIN + '.Executor.' + ll_plan[arg]['type']), ll_plan[arg]['type'])(ll_plan[arg]['type'])
                            exec_obj.execute(self.ll_state, ll_plan[arg]['value'], edge.generated_values)
                            # if str(arg) != 'g_open' and str(arg) != 'g_close':
                                # exec_obj.execute(self.ll_state, ll_plan[arg]['value'], edge.generated_values)
                            # else:
                                # exec_obj.apply(self.ll_state, ll_plan[arg]['value'], edge.generated_values)
                            if str(arg) != 'g_open':
                                gripper_poses.append(self.ll_state.simulator.env.GetRobot('yumi').GetLink('gripper_l_base').GetTransform())
                            phys_val = self.convert_to_ros_traj(ll_plan[arg]['value'], speed=0.1)
                            ros_traj_list.append([ll_plan[arg]['type'], phys_val])
                        for i in edge.effect:
                            if i[0] == 'pos':
                                pred_obj = getattr(importlib.import_module('test_domains.'+Config.DOMAIN + '.Predicates.' + i[1]), i[1])(i[1])
                                pred_obj.apply(self.ll_state, edge.generated_values)
                        self.ll_state.values = self.ll_state.get_values_from_env(self.sim.env)
                    q.append(selected_child)
                else:
                    break
        # Uncomment to execute same sequence as openrave in case of stochastic domains
        # with open(Config.DOMAIN_DIR + 'gripper_poses.pickle', 'wb') as f:
        #     pickle.dump(gripper_poses, f)
        #     f.close()

    def store_refined_ros_tree(self,num_runs):
        stitched_ros_trajectories = []
        for i in range(num_runs):
            update_run_track_file(i+1)
            with open(Config.DOMAIN_DIR + "refined_tree_"+str(i+1)+".pkl", "rb") as traj_dump:
                self.policy_tree = pickle.load(traj_dump)
                traj_dump.close()
            refined_p_tree = RefinedPolicy()
            root = self.policy_tree.get_root()
            new_root = RefinedPolicyNode(root.hl_state)
            refined_p_tree.add_node(new_root)
            queue = [root]
            new_queue = [new_root]
            while len(queue) > 0:
                node = queue.pop()
                new_node = new_queue.pop()
                for child in node.get_children():
                    new_child = RefinedPolicyNode(child.hl_state)
                    edge = self.policy_tree.get_edge(node, child)
                    if edge.has_mp:
                        for arg in edge.exec_seq:
                            edge.ll_plan[arg]['value'] = self.convert_to_ros_traj(edge.ll_plan[arg]['value'], speed=0.1)
                    new_edge = RefinedPolicyEdge(edge.ll_plan, edge.generated_values, edge.has_mp, edge.exec_seq,edge.effect)
                    refined_p_tree.add_node(new_child)
                    refined_p_tree.add_edge(new_node, new_child, new_edge)
                    queue.append(child)
                    new_queue.append(new_child)
            stitched_ros_trajectories.append(refined_p_tree)
        pickle.dump(stitched_ros_trajectories, open(Config.DOMAIN_DIR +"stitched_ros_traj.p", "wb"))

if __name__ == '__main__':
    print("Running Traj")
    choice = input("Runtrajectory/Store trajectory (0/1):  ")
    run_traj = OpenRaveTrajectoryExecutor()
    if choice == 0:
        run_traj.run_trajectory(6)
    elif choice == 1:
        run_traj.store_refined_ros_tree(6)
