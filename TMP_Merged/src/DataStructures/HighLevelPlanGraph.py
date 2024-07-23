from Graph import Graph
from src.DataStructures.HighLevelPlanNode import HighLevelPlanNode
# from InputParser import InputParser
from src.Action.HLAction import HLAction
from src.Parser.ActionConfigParserV2 import ActionConfigParserV2
from src.Parser.pddl_parser import PDDLParser
from src.States.PDDLState import PDDLState
import src.util as util
import copy
import Config
from src.Parser.ObjectTypeMapParser import  ObjecTypeMapParse
from src.DataStructures.Edge import Edge


class HighLevelPlanGraph(Graph):

    def __init__(self,hl_solution=None,problem_specification=None,state_list=None,
                 assume_refinable=False):
        '''
        :param hl_solution: HL Solution Graph with root. Each Node State::Action::Prob
        :param state_list: HL State List
        '''
        super(HighLevelPlanGraph, self).__init__()
        self.hl_solution = hl_solution
        self.edges = {}
        self.state_list = state_list
        self.n = 0
        self.initial_ll_state = None
        #self.__root_node = None
        self.problem_specification = problem_specification
        if self.problem_specification is not None:
            self.object_map = ObjecTypeMapParse(self.problem_specification.pddl_problem_file,problem_specification.pddl_domain_file).get_object_map()
        self.ll_complete_spec_parser = ActionConfigParserV2(Config.LL_ACTION_CONFIG,
                                                            assume_refinable=assume_refinable)
        self.__create_graph(hl_solution)
        self.generated_values = {}
        self.refined_ll_values = {}
        if hl_solution is not None:
            try:
                self.cost = hl_solution.cost
            except Exception,e:
                self.cost = len(self.edges)

        else:
            self.cost = float("inf")

    def store_refined_ll_values(self,parent_node,child_node,refined_ll_values):
        if parent_node not in self.refined_ll_values:
            self.refined_ll_values[parent_node]  = {}
        self.refined_ll_values[parent_node][child_node] = refined_ll_values

    def get_refined_ll_values(self,parent_node,child_node):
        if parent_node in self.refined_ll_values:
            if child_node in self.refined_ll_values[parent_node]:
                return self.refined_ll_values[parent_node][child_node]
        return None

    def store_mp(self,parent_node,child_node,mp):
        if parent_node not in self.generated_values:
            self.generated_values[parent_node] = {}
        self.generated_values[parent_node][child_node] = mp

    def get_mp(self,parent_node,child_node):
        if parent_node in self.generated_values:
            if child_node in self.generated_values[parent_node]:
                return self.generated_values[parent_node][child_node]
        return None

    def is_refined(self, hl_action_node):
        if hl_action_node in self.refined_ll_values:
            if len(self.refined_ll_values[hl_action_node].keys()) == len(hl_action_node.get_children()):
                return True
            else:
                return False
        else:
            return False

    def readjust_probabilities(self):
        root = self.get_root()
        q = [root]
        p = 1
        while len(q) > 0:
            cur = q.pop(0)
            children = cur.get_children()
            for child in children:
                parent_of_parent = cur.get_parent()
                edge = self.get_edge(cur,child)
                if parent_of_parent is None:
                    edge.prob = edge.prob * p
                else:
                    parent_edge = self.get_edge(parent_of_parent,cur)
                    edge.prob = edge.prob * parent_edge
                q.append(child)



    def add_edge(self,parent,child,edge,label=None,new=True,only_nx=False):
        super(HighLevelPlanGraph,self).add_edge(parent,child,label)
        if not only_nx:
            if new:
                e = edge
                self.edges[parent][child] = e


    def remove_edge(self,parent,child):
        super(HighLevelPlanGraph,self).remove_edge(parent,child)
        self.edges[parent].pop(child,None)

    def add_node(self,node,new=True):
        super(HighLevelPlanGraph,self).add_node(node)
        if new:
            self.edges[node] = {}

    def remove_node(self,node):
        super(HighLevelPlanGraph,self).remove_node(node)
        self.edges.pop(node)

    def __deepcopy__(self, memodict={}):
        '''
        resolve copy issue
        :param memodict:
        :return:
        '''

        new_hlpg = HighLevelPlanGraph()
        new_hlpg.generated_values = copy.deepcopy(self.generated_values)
        new_hlpg.refined_ll_values = copy.deepcopy(self.refined_ll_values)
        hl_solution_copy = copy.deepcopy(self.hl_solution)
        state_list_copy = copy.deepcopy(self.state_list)
        n_copy = copy.deepcopy(self.n)
        init_state_copy = copy.deepcopy(self.initial_ll_state)
        problem_specification_copy = copy.deepcopy(self.problem_specification)
        object_map_copy = copy.deepcopy(self.object_map)

        new_hlpg.hl_solution = hl_solution_copy
        new_hlpg.state_list = state_list_copy
        new_hlpg.n = n_copy
        new_hlpg.initial_ll_state = init_state_copy
        new_hlpg.problem_specification = problem_specification_copy
        new_hlpg.object_map = object_map_copy

        new_hlp_root_node = copy.deepcopy(self.get_root(),{"type" : 1})

        new_hlpg.set_root(new_hlp_root_node)
        new_hlpg.add_node(new_hlp_root_node)
        new_hlpg.object_map=copy.deepcopy(self.object_map)

        copy_list = [new_hlp_root_node]
        old_list = [self.get_root()]
        new_hlpg.cost = copy.deepcopy(self.cost)
        while len(old_list) > 0:
            cur_old = old_list.pop(0)
            cur_new = copy_list.pop(0)
            children_old = cur_old.get_children()
            if children_old is not None:
                for child in children_old:
                    child_copy = copy.deepcopy(child,{"type" : 1})
                    new_hlpg.add_node(child_copy,new=True)
                    e = copy.deepcopy(self.edges[cur_old][child])
                    e.add_parent(cur_new)
                    e.add_child(child_copy)
                    # if cur_new not in new_hlpg.edges:
                    #     new_hlpg.edges[cur_new] = {}
                    # new_hlpg.edges[cur_new][child_copy] = e
                    new_hlpg.add_edge(cur_new, child_copy,e, new=True)
                    cur_new.add_child(child_copy)
                    child_copy.set_parent(cur_new)
                    copy_list.append(child_copy)
                    old_list.append(child)
        return new_hlpg

    def __create_graph(self,hl_solution):
        if hl_solution is not None:
            root = hl_solution.get_root_node()
            root_graph_node,edge = self.make_high_level_node(root)
            self.add_node(root_graph_node)
            self.set_root(root_graph_node)
            queue = [(root,root_graph_node,edge)]
            while len(queue) > 0:
                cur,cur_graph_node,edge = queue.pop(0)
                for node in self.hl_solution.policyTree[cur]:
                    edge_copy = copy.deepcopy(edge)
                    new_graph_node,new_edge = self.make_high_level_node(node)
                    cur_graph_node.add_child(new_graph_node)
                    new_graph_node.set_parent(cur_graph_node)
                    edge_copy.add_child(new_graph_node)
                    edge_copy.prob = new_edge.prob
                    self.add_node(new_graph_node)
                    self.add_edge(cur_graph_node,new_graph_node,edge_copy)
                    queue.append((node,new_graph_node,new_edge))




    def make_high_level_state(self,state_str):
        pos_set = set()
        tokens = state_str.split(":")
        for token in tokens[1:]:
            temp = token.split(")")[0]
            temp += ")"
            pos_set.add(temp)
        return PDDLState(trueSet = pos_set)

    def make_high_level_node(self,node):
        detial_node = self.hl_solution.policyTree.nodes[node]

        details = detial_node['label'].strip().split("::")
        if self.state_list is None:
            state = self.make_high_level_state(details[0])
        else:
            state = self.state_list[self.n]
            self.n+=1
            
        if "tmp_goal_reached" in details[1]:
            actionStr= "tmp_goal_reached"
        elif 'STOP' not in details[1]:
            actionStr = details[1].strip()[1:-1]
        else:
            actionStr = "STOP"

        prob = float(details[2].strip().replace("\"",""))
        args = actionStr.lower().split()
        actionName = args[0]
        arg_list = args[1:]
        ll_complete_spec_parser = self.ll_complete_spec_parser
        try:
            parser = PDDLParser(self.problem_specification.pddl_domain_file,
                                self.problem_specification.pddl_problem_file)
            list_argument_objects, precondition_pos, precondition_neg, effect_pos, effect_neg = parser.parse_action(
                actionName)
        except:
            print "PDDL Parser ERROR! Could be due to forall quantifier, setting preconditions and effects to None"
            precondition_neg = None
            precondition_pos = None
            effect_neg = None
            effect_pos = None

        list_argument_objects = args[1:]
        list_argument_objects = [x.lower() for x in list_argument_objects]
        action_arg_map = self.get_action_arg_map(list_argument_objects)
        ll_action_spec = copy.deepcopy(ll_complete_spec_parser.get_specification(actionName))
        hlAction = HLAction(actionName,precondition_pos,precondition_neg,effect_pos,effect_neg,action_arg_map)
        assert  ll_action_spec is not None \
            or actionName == "tmp_goal_reached" \
            or actionName in ll_complete_spec_parser.get_ignore_action_list(), "Could not find Action Spec for action: " + str(actionName)

        if ll_action_spec is not None:
            ll_action_spec.generated_values = {}
            for i in range(len(arg_list)):
                ll_action_spec.generated_values[ll_action_spec.hl_args[i]] = arg_list[i]
            ll_action_spec.init_values = copy.deepcopy(ll_action_spec.generated_values)
        node = HighLevelPlanNode(hlAction, hl_state=state,children= [])
        e = Edge(parent=node,child=None,hl_action=hlAction,ll_action_spec=ll_action_spec,prob=prob,debug_name=actionName)
        
        # Check if this node is a goal node.
        if actionStr == "tmp_goal_reached":
            
            node.is_goal = True
        
        return node,e

    def get_action_arg_map(self,action_arguments):
        temp = {}
        for arg in action_arguments:
            for t in self.object_map:
                if arg in self.object_map[t]:
                    if t in temp:
                        temp[t].append(arg)
                    else:
                        temp[t] = [arg]
        return temp

    def reset_child_generators(self):
        children = [self.get_root()]

        while children:
            child = children.pop(0)
            children.extend(child.get_children())
            child.reset_child_generator()

    def get_edge(self,parent,child):
        if parent in self.edges:
            if child in self.edges[parent]:
                return self.edges[parent][child]
        return None

    def add_sub_tree(self,parent,new_tree,edge):
        super(HighLevelPlanGraph,self).add_sub_tree(parent,new_tree)
        self.edges.update(new_tree.edges)
        self.edges[parent][new_tree.get_root()] = edge
        edge.add_parent(parent)
        edge.add_child(new_tree.get_root())


    def merge(self,parent,new_tree,failed_node,label= ""):
        edge_copy = copy.deepcopy(self.get_edge(parent,failed_node))
        parent_of_parent = parent.get_parent()
        if parent_of_parent is not None:
            parent_edge_prob = self.get_edge(parent_of_parent,parent).prob

            new_tree_root = new_tree.get_root()
            if parent_of_parent is not None:
                q = [new_tree_root]
                while len(q) > 0:
                    cur = q.pop(0)
                    children = cur.get_children()
                    for child in children:
                        edge = new_tree.get_edge(cur,child)
                        edge.prob  *= parent_edge_prob
                        q.append(child)
        parent.remove_child(failed_node)
        self.remove_edge(parent,failed_node)
        failed_node.set_parent(None)
        # self.remove_node(failed_node)
        parent.add_child(new_tree.get_root())
        parent.reset_child_generator()
        new_tree.get_root().set_parent(parent)
        self.add_sub_tree(parent,new_tree,edge_copy)
        copy_policy_tree = self.__deepcopy__()
        parent.remove_child(new_tree.get_root())
        parent.add_child(failed_node)
        self.remove_edge(parent,new_tree.get_root())
        self.add_edge(parent,failed_node,edge_copy)
        failed_node.set_parent(parent)
        copy_policy_tree.cost += new_tree.cost
        return copy_policy_tree

    def recalculate_all_edges(self):
        self.clear_graph()
        current_node = self.get_root()
        self.add_node(current_node,new=False)
        queue = [current_node]
        while queue:
            current = queue.pop(0)
            for node in current.get_children():
                self.add_node(node,new=False)
                self.add_edge(current,node,None,new=False,only_nx=True)
                queue.append(node)

