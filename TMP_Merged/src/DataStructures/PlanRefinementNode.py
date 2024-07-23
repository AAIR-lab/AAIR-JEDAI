from src.DataStructures.GraphNode import GraphNode
import sys
from src.Planner import PlannerFactory
import random
import Config
from Queue import PriorityQueue
from src.DataStructures.HighLevelActionSequence import HighLevelAcionSequnce
import networkx as nx
from src.tmp_exceptions import FailedPredicateException, TimeOutException, OutOfPossibleErrorsException
import copy
from src.DataStructures.RefinedPolicy import RefinedPolicy
from src.DataStructures.RefinedPolicyNode import RefinedPolicyNode
from src.DataStructures.RefinedPolicyEdge import RefinedPolicyEdge
import pickle
import time
import importlib
from src.States.OpenRaveLowLevelState import OpenRaveLowLevelState
import tempfile
import os
from PIL import Image
import numpy as np

class PlanRefinementNode(GraphNode):
    def __init__(self, problem_specification=None,hl_plan_graph=None, hl_plan_tree= None, parent=None, last_refined_hl_plan_graph_node=None, \
                 lqueue = None,ll_state_values=None,mode = "err_free",visited = True,start_time = None,restart = 0, action_execute = 0):
        super(PlanRefinementNode,self).__init__()
        self.__parent = parent
        self.__children = []
        self.problem_specification = problem_specification
        self.hl_plan_graph = hl_plan_graph
        self.lqueue = lqueue
        self.next_child_count = 0
        self.hl_plan_tree = hl_plan_tree
        self.hl_state_type = Config.HL_STATE_TYPE
        self.last_refined_hl_plan_graph_node = last_refined_hl_plan_graph_node
        self.ll_state_values = ll_state_values
        self.mode = mode
        self.visited = visited
        self.start_time = start_time
        self.last_refined_hl_action_node = None
        self.child_queue = None
        self.restart = restart
        self.action_execute = action_execute
        self.refined_mass = 0
        self.start_time = None



    def prepare_queue(self,new = False):
        if self.lqueue is None or new:
            root = self.hl_plan_tree.get_root()
            self.lqueue = PriorityQueue()
            total_prob = 0
            for node in self.hl_plan_tree.graph.nodes:
                if self.hl_plan_tree.graph.out_degree[node] == 0:
                    path = nx.shortest_path(self.hl_plan_tree.graph,root,node)
                    last_edge = self.hl_plan_tree.get_edge(path[-2],path[-1])
                    if last_edge.ll_plan is not None:
                        continue
                    hlas = HighLevelAcionSequnce(path)
                    prob = last_edge.prob
                    total_prob += prob
                    p = len(path) / float(prob)
                    if Config.PC:
                        self.lqueue.put((p,hlas))
                    else:
                        self.lqueue.put((random.random(),hlas))
        if self.lqueue.qsize() > 1:
            pass

        return self.lqueue

    def set_parent(self, node):
        self.__parent = node

    def set_hl_plan_graph(self, hl_plan_graph):
        self.hl_plan_graph = hl_plan_graph

    def get_children(self):
        return self.__children
    def get_parent(self):
        return self.__parent

    def add_child(self, child):
        self.__children.append(child)

    def try_refine(self,hl_action_node=None, mode="err_free", resource_limit=10,force_start_from_begining=False,n_actions = 2):
        while True:
            if self.start_time is not None:
                self.start_time = time.time()
            if time.time() - self.start_time > Config.MAX_TIME:
                self.start_time = None
                raise TimeOutException(hl_action_node=hl_action_node)
            if resource_limit == 0:
                return False,"timeout",None
            reset_ll_sepc = False
            if hl_action_node is None:
                self.hl_plan_graph.reset_child_generators()
                hl_action_node = self.hl_plan_graph.get_root()
                reset_ll_sepc = True
            hl_action_node.hlpg_node_ref.ll_state.sync_simulator()
            child = hl_action_node.generate_child()
            stop = False
            if child is not None and not stop:
                edge = self.hl_plan_tree.get_edge(hl_action_node.hlpg_node_ref,child.hlpg_node_ref)
            else:
                if child is not None:
                    child.set_ll_state(copy.deepcopy(edge.refined_ll_state))
                    child.hlpg_node_ref.set_ll_state(copy.deepcopy(edge.refined_ll_state))
                print "Finished for this sequence..."
                return True,None,None

            if edge.ll_action_spec is None:
                ll_state = copy.deepcopy(hl_action_node.hlpg_node_ref.ll_state)
                edge.refined_ll_state = ll_state
                child.set_ll_state(ll_state)
                child.hlpg_node_ref.set_ll_state(ll_state)
                hl_action_node = child
                # n_actions -= 1
                continue

            if edge.ll_plan is None:
                if reset_ll_sepc:
                    edge.ll_action_spec.reset_argument_generators()
                if hl_action_node.get_parent() is None:
                    resource_limit -= 1
                hl_action = edge.hl_action
                failed_predicates = None
                ll_complete_action_spec =  edge.ll_action_spec
                hl_state = hl_action_node.hl_state
                child_hl_state = child.hl_state
                ll_complete_action_spec.generated_values["current_hl_state"] = hl_state
                ll_complete_action_spec.generated_values["next_hl_state"] = child_hl_state
                ll_state = copy.deepcopy(hl_action_node.hlpg_node_ref.ll_state)
                ll_state_values = ll_state.get_values_from_env()
                hl_action_node.init_ll_values = copy.deepcopy(ll_state_values)
                hl_action_node.hlpg_node_ref.init_ll_values = copy.deepcopy(ll_state_values)
                ll_argument_generator = ll_complete_action_spec.generator(low_level_state=ll_state,
                                                                          backtrack_on_failure=(mode == 'err_free'),
                                                                          start_time=self.start_time)
                try:
                    result,generated_values, next_ll_state = ll_argument_generator.next()
                    ll_plan = result#TODO
                    edge.has_mp = True
                    edge.ll_plan = ll_plan
                    edge.generated_values = generated_values
                    edge.refined_ll_values = copy.deepcopy(ll_state.get_values_from_env())
                    edge.refined_ll_state = copy.deepcopy(next_ll_state)
                    self.stable_ll_state = copy.deepcopy(next_ll_state)
                    self.last_refined_hl_action_node = hl_action_node
                    child.set_ll_state(copy.deepcopy(next_ll_state))
                    child.hlpg_node_ref.set_ll_state(copy.deepcopy(next_ll_state))
                    hl_action_node = child
                    self.start_time = time.time()
                    # n_actions -= 1
                except FailedPredicateException as failed_predicate_excep:
                    # ll_state = ll_state_cpy
                    ll_state.values = copy.deepcopy(ll_state_values)
                    ll_state.sync_simulator(ll_state_values)
                    failed_predicate = failed_predicate_excep.predicate
                    if failed_predicate is not None:
                        failed_predicate.set_proposition_string(failed_predicate_excep.failure_reason)
                    last_successful_state = failed_predicate_excep.last_successful_state
                    print "Failed At: " + str(hl_action)
                    return False, failed_predicate, hl_action_node

                except OutOfPossibleErrorsException:
                    # ll_state = ll_state_cpy
                    ll_state.values = copy.deepcopy(ll_state_values)
                    ll_state.sync_simulator()
                    ll_complete_action_spec.reset_argument_generators()
                    return False, None, hl_action_node

                except TimeOutException:
                    raise TimeOutException(hl_action_node=hl_action_node)
            else:
                if child.ll_state is None:
                    child.set_ll_state(copy.deepcopy(edge.refined_ll_state))
                    child.hlpg_node_ref.set_ll_state(copy.deepcopy(edge.refined_ll_state))
                    edge.refined_ll_state.sync_simulator()
                hl_action_node = child

    def simulate_execution(self, hl_parent_node, hl_child_node,
                           max_samples_per_sequence=25,
                           sample_collection_interval_in_ms=20):
        
        edge = self.hl_plan_tree.get_edge(hl_parent_node, hl_child_node)
        
        simulation = []
        assert not hl_parent_node.goal
                
        if not edge.has_mp:
            
            simulation.append((hl_parent_node.ll_state.values, hl_parent_node.hl_state.getTrueProps()))
            simulation.append((hl_child_node.ll_state.values, hl_child_node.hl_state.getTrueProps()))
            return simulation
        else:
            
            assert edge.ll_action_spec.exec_sequence is not None

        current_ll_state = OpenRaveLowLevelState()
        current_ll_state.values = hl_parent_node.ll_state.values
        current_ll_state.sync_simulator(hl_parent_node.ll_state.values)

        for arg in edge.ll_action_spec.exec_sequence:
            exec_obj = getattr(importlib.import_module( Config.TEST_DIR_NAME + '.'+Config.DOMAIN + '.Executor.' + edge.ll_plan[arg]['type']), edge.ll_plan[arg]['type'])(edge.ll_plan[arg]['type'])
            ll_state_values = exec_obj.execute(
                current_ll_state, 
                edge.ll_plan[arg]['value'], 
                edge.generated_values,
                sleep_interval_in_ms=sample_collection_interval_in_ms)
            
            # Update the simulator state.
            current_ll_state.values = current_ll_state.get_values_from_env(
                current_ll_state.simulator.env)
            
            ll_indicies = range(len(ll_state_values))
            random_indicies = sorted(
                random.sample(
                    ll_indicies,
                    k=min(max_samples_per_sequence, len(ll_state_values))))
            
            # Add a few from the subsequence
            for idx in random_indicies:
                
                simulation.append((ll_state_values[idx], hl_parent_node.hl_state.getTrueProps()))
        
        # Last one is always added
        simulation.append((hl_child_node.ll_state.values, hl_child_node.hl_state.getTrueProps()))

        return simulation

    def get_py3_pickable_version(self, refined_policy_tree):
        
        # Need to convert everything to raw types
        # python3 pickable version needs an encoding too.
        
        py3_dict = {}
        py3_dict["tmp_py3_policy"] = True
        py3_dict["has_simulation"] = refined_policy_tree.has_simulation
        py3_dict["is_goal_reachable"] = refined_policy_tree.is_goal_reachable
        
        fringe = [(0, refined_policy_tree.get_root())]
        
        idx = 1
        while len(fringe) > 0:
            
            node_id, node = fringe.pop()
            child_ids = []
            probabilities = []
            for child in node.get_children():
                
                
                child_ids.append(idx)
                
                edge = refined_policy_tree.get_edge(node, child)
                assert edge.prob is not None
                probabilities.append(edge.prob)
                
                fringe.append((idx, child))
                idx += 1
                
            # Normalize the probabilities
            total_probability_mass = 1.0 * sum(probabilities) 
            probabilities = [prob / total_probability_mass for prob in probabilities]
                
            py3_dict[node_id] = {
                
                "is_goal": node.is_goal,
                "child_ids": child_ids,
                "probabilities": probabilities,
                "ll_state": node.ll_state.values,
                "hl_state": node.hl_state.getTrueProps(),
                "simulation": node.simulation,
                "image": np.asarray(node.image),
            }
                
        return py3_dict

    def make_picklable(self, refined_policy_tree):
        
        fringe = [refined_policy_tree.get_root()]
        while len(fringe) > 0:
            
            node = fringe.pop()
            
            # Reset the ll_state
            node.ll_state = None
            
            fringe += node.get_children()

        
    def set_camera_for_screenshot(self, viewer, ll_state):
        
        for obj_name in ll_state.values["objects"].keys():
            if "yumi" in obj_name:
                
                viewer.SetCamera([
                    [ 0.12983982,  0.26632414, -0.95509846,  1.64310551],
                    [ 0.99015579,  0.01596189,  0.13905654, -0.2587153 ],
                    [ 0.05227929, -0.96375135, -0.2616299 ,  0.61909211],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]])
                break
                
    def get_camera_image(self, ll_state):
        
        env = ll_state.simulator.env
        viewer = env.GetViewer()
        
        self.set_camera_for_screenshot(viewer, ll_state)
        temp_file = tempfile.NamedTemporaryFile()
        
        # First step is to make the viewer the active window.
        # This should allow screen capture software to directly
        # get the screenshot without worrying about setting the
        # active window.
        os.system("wmctrl -a OpenRAVE")
        os.system("gnome-screenshot -w -f %s" % (temp_file.name))
        
        image = Image.open(temp_file.name)
        temp_file.close()
        
        return image

    def store_refined_tree(self,store=False, store_simulated_executions=False,
                           output_dir=None, store_image=False):
        refined_p_tree = RefinedPolicy()
        
        refined_p_tree.has_simulation = store_simulated_executions
        refined_p_tree.is_goal_reachable = False
        
        root = self.hl_plan_tree.get_root()
        
        new_root = RefinedPolicyNode(root.hl_state)
        new_root.simulation = None
        
        refined_p_tree.add_node(new_root)
        queue = [root]
        new_queue = [new_root]
        
        while len(queue) > 0:
            node = queue.pop()
            new_node = new_queue.pop()
            
            new_node.ll_state = node.ll_state
            if store_image:
                new_node.image = self.get_camera_image(node.ll_state)
            else:
                new_node.image = None
           
            # Copy over the goal info first.
            new_node.is_goal = node.is_goal
            refined_p_tree.is_goal_reachable |= new_node.is_goal 
            
            for child in node.get_children():
                new_child = RefinedPolicyNode(child.hl_state)
                edge = self.hl_plan_tree.get_edge(node, child)
                all_effects = []
                if edge.ll_action_spec is not None:
                    # TODO Find a better way of storing effects
                    exec_seq = edge.ll_action_spec.exec_sequence
                    # effect = edge.ll_action_spec.effect
                    # if effect.getPositivePredicates() is not None:
                    #     for e in effect.getPositivePredicates():
                    #         all_effects.append(['pos',e.name])
                    # if effect.getNegativePredicates() is not None:
                    #     for e in effect.getNegativePredicates():
                    #         all_effects.append(['neg',e.name])
                    if edge.generated_values is not None:
                        all_effects = edge.generated_values["next_hl_state"].getAllProps()
                    else:
                        all_effects = None
                else:
                    exec_seq = None
                    effect = None
                    
                new_edge = RefinedPolicyEdge(edge.ll_plan, 
                                             edge.generated_values, 
                                             edge.has_mp, 
                                             exec_seq,
                                             all_effects,
                                             edge.refined_ll_values,
                                             prob=edge.prob)

                refined_p_tree.add_node(new_child)
                refined_p_tree.add_edge(new_node, new_child, new_edge)
                queue.append(child)
                new_queue.append(new_child)
                
                if store_simulated_executions:
                    
                    simulation = self.simulate_execution(node, child)
                    new_child.simulation = simulation
                else:
                    new_child.simulation = None
                    

                    
        if store:
            
            py3_dict = self.get_py3_pickable_version(refined_p_tree)
            
            if output_dir is None:
                output_dir = Config.DOMAIN_DIR
                
                
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            else:
                assert os.path.isdir(output_dir)
                
            pickle.dump(py3_dict, open(output_dir +"/refined_tree.py3.pkl", "wb"))
            

            self.make_picklable(refined_p_tree)
            pickle.dump(refined_p_tree, open(output_dir +"/refined_tree.pkl", "wb"))
            

        return refined_p_tree

# def try_refine(self, hl_action_node=None, mode='err_free', resource_limit=10 ,force_start_from_beginning=False):
    #     '''
    #     :return success, failure_node
    #     '''
    #
    #     while True:
    #         if resource_limit == 0:
    #             return False,"timeout",None
    #
    #         if hl_action_node is None:
    #             self.hl_plan_graph.reset_child_generators()
    #             hl_action_node = self.hl_plan_graph.get_root()
    #
    #             if hl_action_node.hlpg_node_ref.ll_plan is None:
    #                 if hl_action_node.hlpg_node_ref.ll_action_spec:
    #                     hl_action_node.hlpg_node_ref.ll_action_spec.reset_argument_generators()
    #             hl_action_node.hlpg_node_ref.ll_state.sync_simulator()
    #         else:
    #             if hl_action_node.get_parent() is not None:
    #                 hl_action_node.get_parent().hlpg_node_ref.ll_state.sync_simulator(self.hl_plan_tree.get_refined_ll_values(hl_action_node.hlpg_node_ref.get_parent(),hl_action_node.hlpg_node_ref))
    #
    #
    #         if hl_action_node.hlpg_node_ref.ll_action_spec is None: #If there is no corresponding ll_action spec for the action, don't refine
    #             ll_state = hl_action_node.ll_state
    #             hl_action_node = hl_action_node.generate_child()
    #             if hl_action_node is None:
    #                 return True, None, None
    #
    #             hl_action_node.set_ll_state(ll_state)
    #             continue;
    #
    #         if not self.hl_plan_tree.is_refined(hl_action_node.hlpg_node_ref):
    #
    #             if hl_action_node.get_parent() is None:
    #                 resource_limit = resource_limit - 1
    #
    #             hl_action = hl_action_node.hl_action
    #
    #             hl_action.hl_arguments['robot'] = Config.ROBOT_NAME
    #             print hl_action
    #             # #######TEST############
    #             # if hl_action.actionName == 'moveto':
    #             #     hl_action.hl_arguments = {u'object_name':'object0'}
    #             #
    #             # elif hl_action.actionName == 'grasp':
    #             #     hl_action.hl_arguments = {u'object_name':'object0', u'robot':'fetch'}
    #             #
    #             # ######################
    #
    #             failed_predicates = None
    #             ll_complete_action_spec = hl_action_node.hlpg_node_ref.ll_action_spec
    #             ll_state = copy.deepcopy(hl_action_node.ll_state)
    #             hl_action_node.hlpg_node_ref.set_ll_state(copy.deepcopy(ll_state))
    #             current_hl_state = hl_action_node.hlpg_node_ref.hl_state
    #             child = hl_action_node.generate_child()
    #             if child is not None:
    #                 next_hl_state = child.hlpg_node_ref.hl_state
    #             else:
    #                 next_hl_state = None
    #             ll_complete_action_spec.generated_values["current_hl_state"] = current_hl_state
    #             ll_complete_action_spec.generated_values["next_hl_state"] = next_hl_state
    #
    #             ll_argument_generator = ll_complete_action_spec.generator(low_level_state=ll_state,
    #                                                                       backtrack_on_failure=(mode == 'err_free'), start_time = self.start_time)
    #             ll_state_values = ll_state.get_values_from_env()
    #             ll_state.set_values(ll_state_values)
    #             hl_action_node.init_ll_values = copy.deepcopy(ll_state_values)
    #             hl_action_node.hlpg_node_ref.init_ll_values = copy.deepcopy(ll_state_values)
    #             try:
    #                 result,generated_values, next_ll_state = ll_argument_generator.next() #TODO need resulting_state and ll_plan
    #                 # NOTE: Once we have all the argument values, update the low level state, simulate forward in time.
    #                 # Currently this is implicitly done during argument generation
    #
    #                 ll_plan = result#TODO
    #                 hl_action_node.has_mp = True
    #                 hl_action_node.hlpg_node_ref.has_mp = True
    #
    #                 # if mode == "partial_traj":
    #                 #     raise FailedPredicateException(None,failure_reason = None)
    #
    #                 hl_action_node.hlpg_node_ref.save(ll_plan, next_ll_state)
    #                 hl_action_node.save(ll_plan, next_ll_state)
    #                 hl_action_node.hlpg_node_ref.generated_values = generated_values
    #                 # hl_action_node.generated_values = generated_values
    #
    #                 refined_ll_values = copy.deepcopy(ll_state.get_values_from_env(ll_state.simulator.env))
    #                 self.hl_plan_tree.store_mp(hl_action_node.hlpg_node_ref,child.hlpg_node_ref,generated_values)
    #                 self.hl_plan_tree.store_refined_ll_values(hl_action_node.hlpg_node_ref,child.hlpg_node_ref,refined_ll_values)
    #                 # hl_action_node.hlpg_node_ref.refined_ll_values = copy.deepcopy(ll_state.get_values_from_env(ll_state.simulator.env))
    #
    #                 self.stable_ll_state = copy.deepcopy(next_ll_state)
    #                 self.last_refined_hl_action_node = hl_action_node
    #
    #
    #                 # child = hl_action_node.generate_child()
    #                 if child is not None:
    #                     child.set_ll_state(copy.deepcopy(next_ll_state))
    #                     child.hlpg_node_ref.set_ll_state(copy.deepcopy(next_ll_state))
    #                     hl_action_node = child
    #
    #                     # return self.try_refine(child, mode, resource_limit)
    #                 else:
    #                     print("Finished for this sequence...")
    #                     return True, None,None  # "Finished"
    #
    #             except FailedPredicateException as failed_predicate_excep:
    #                 # ll_state = ll_state_cpy
    #                 ll_state.values = copy.deepcopy(ll_state_values)
    #                 ll_state.sync_simulator(ll_state_values)
    #                 failed_predicate = failed_predicate_excep.predicate
    #                 if failed_predicate is not None:
    #                     failed_predicate.set_proposition_string(failed_predicate_excep.failure_reason)
    #                 last_successful_state = failed_predicate_excep.last_successful_state #TODO need last_successful_state
    #                 print "Failed At: " + str(hl_action)
    #                 return False, failed_predicate, hl_action_node
    #
    #             except OutOfPossibleErrorsException:
    #                 # ll_state = ll_state_cpy
    #                 ll_state.values = copy.deepcopy(ll_state_values)
    #                 ll_state.sync_simulator()
    #                 hl_action_node.hlpg_node_ref.ll_action_spec.reset_argument_generators()
    #                 return False, None, hl_action_node
    #
    #             except TimeOutException:
    #                 raise TimeOutException(hl_action_node=hl_action_node)
    #
    #         else:
    #             child = hl_action_node.generate_child()
    #             if child is not None:
    #                 if child.ll_state is None:
    #                     child.set_ll_state(copy.deepcopy(hl_action_node.hlpg_node_ref.ll_state))
    #                     # child.hlpg_node_ref.set_ll_state(hl_action_node.hlpg_node_ref.ll_state)
    #                 ll_state_values = hl_action_node.hlpg_node_ref.refined_ll_values
    #                 ll_state = hl_action_node.ll_state
    #                 ll_state.sync_simulator(ll_state_values)
    #                 hl_action_node = child
    #