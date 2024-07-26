from src.DataStructures.HighLevelPlanNode import HighLevelPlanNode
# from InputParser import InputParser
from src.Action.HLAction import HLAction
from src.Parser.pddl_parser import PDDLParser

import src.util as util
import copy
import Config



from Graph import Graph


class HighLevelAcionSequnce(Graph):

    def __init__(self,hlg_node_list):

        super(HighLevelAcionSequnce,self).__init__()
        self.__root_node = copy.deepcopy(hlg_node_list[0])
        self.set_root(self.__root_node)
        self.length = len(hlg_node_list)
        self.action_list = hlg_node_list
        prev = self.__root_node
        prev.hlpg_node_ref = hlg_node_list[0]
        if prev.hlpg_node_ref is None:
            print "None Here"

        for node in hlg_node_list[1:]:
            for i in prev.get_children():
                if i not in hlg_node_list:
                    prev.add_other_child(copy.deepcopy(i))
            prev.set_children([])
            new_node = copy.deepcopy(node,{"partial_copy":True})
            new_node.hlpg_node_ref = node
            if new_node.hlpg_node_ref is None:
                print "None in the loop"
            self.add_node(new_node)
            prev.add_child(new_node)
            new_node.set_parent(prev)
            self.add_edge(prev,new_node)
            prev = new_node

    def reset_child_generators(self):
        children = [self.get_root()]

        while children:
            child = children.pop(0)
            children.extend(child.get_children())
            child.reset_child_generator()

    def get_root(self):
        return self.__root_node

    # def __str__(self):
    #     s = ""
    #     t = self.__root_node
    #     while t is not None:
    #         s += t.