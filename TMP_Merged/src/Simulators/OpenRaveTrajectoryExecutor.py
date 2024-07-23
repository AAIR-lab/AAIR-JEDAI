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
import Config
import src.util as util
import pickle
from src.Simulators.OpenRaveSimulator import *

class OpenRaveTrajectoryExecutor:
    def __init__(self,policy_tree=None,ll_state=None):
        if policy_tree is None:
            with open(Config.DOMAIN_DIR + "refined_tree.pkl", "rb") as traj_dump:
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
        stochastic_policy = getattr(importlib.import_module( Config.TEST_DIR_NAME + '.'+Config.DOMAIN + '.StochasticPolicy'),'StochasticPolicy')
        return stochastic_policy.select_child_using_props(policy_tree, node, choice, children)


    def convert_to_ros_traj(self, val, robot_name, speed=0.1):
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
                    return util.ros_manip_trajectory_from_openrave(self.ll_state.simulator.env.GetRobot(robot_name), trajobj, velocity_scale=speed)
                elif dct['planning_method'] == 'PlanToBasePose':
                    return util.ros_base_trajectory_from_openrave(self.ll_state.simulator.env.GetRobot(robot_name), sim, trajobj)
            else:
                return val
        else:
            return val

    def store_refined_ros_tree(self):
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
                        edge.ll_plan[arg]['value'] = self.convert_to_ros_traj(edge.ll_plan[arg]['value'], edge.generated_values['robot'], speed=0.1)
                new_edge = RefinedPolicyEdge(edge.ll_plan, edge.generated_values, edge.has_mp, edge.exec_seq,edge.effect)
                refined_p_tree.add_node(new_child)
                refined_p_tree.add_edge(new_node, new_child, new_edge)
                queue.append(child)
                new_queue.append(new_child)
        pickle.dump(refined_p_tree, open(Config.DOMAIN_DIR +"ros_traj.p", "wb"))

    def run_trajectory(self):
        # This function executes the trajectory on openrave and also creates ros trajectories
        ros_traj_list = []
        root = self.policy_tree.get_root()
        q = [root]
        ros_traj_list = []
        while len(q) > 0:
            node = q.pop()
            children = node.get_children()
            if len(children) > 0:
                # Will only enter here for mdp problems
                if len(children) > 1:
                    choice = int(raw_input("Choose succes/ failure (1,0): "))
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
                        exec_obj = getattr(importlib.import_module( Config.TEST_DIR_NAME + '.'+Config.DOMAIN + '.Executor.' + ll_plan[arg]['type']), ll_plan[arg]['type'])(ll_plan[arg]['type'])
                        exec_obj.execute(self.ll_state, ll_plan[arg]['value'], edge.generated_values)
                        # phys_val = self.convert_to_ros_traj(ll_plan[arg]['value'], speed=0.1)
                        #ros_traj_list.append([ll_plan[arg]['type'], phys_val])
                    # for i in edge.effect:
                    #     # import IPython
                    #     # IPython.embed()
                    #     # if i[0] == 'pos':
                    #     #     pred_obj = getattr(importlib.import_module(  Config.TEST_DIR_NAME + '.'+Config.DOMAIN + '.Predicates.' + i[1]), i[1])(i[1])
                    #     #     pred_obj.apply(self.ll_state, edge.generated_values)
                    #     import IPython
                    #     IPython.embed()
                    self.ll_state.sync_simulator(edge.ll_values)
                    self.ll_state.values = self.ll_state.get_values_from_env(self.sim.env)
                q.append(selected_child)
            else:
                break
        # Uncomment to execute same sequence as openrave in case of stochastic domains
        # with open(Config.DOMAIN_DIR + 'ros_trajectory.pickle', 'wb') as f:
        #     pickle.dump(ros_traj_list, f)
        #     f.close()


if __name__ == '__main__':
    print("Running Traj")
    choice = input("Runtrajectory/Store trajectory (0/1):  ")
    run_traj = OpenRaveTrajectoryExecutor()
    if choice == 0:
        run_traj.run_trajectory()
    elif choice == 1:
        run_traj.store_refined_ros_tree()
