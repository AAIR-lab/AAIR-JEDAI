from Graph import  Graph
from LowLevelSequenceNode import LowLevelSequenceNode
from src.Generators import *

import copy
import src.util as util

class LowLevelDependencyGraph(Graph):
    def __init__(self, ll_gen_sequence=None, arg_to_generator_type_map=None, depender_map={}, dependee_map={}, gen_type_to_class_map=None ):
        super(LowLevelDependencyGraph, self).__init__()
        self.arg_to_node_map = {}


        if ll_gen_sequence is not None:
            self.__createGraph(ll_gen_sequence, arg_to_generator_type_map, depender_map, dependee_map, gen_type_to_class_map)

    def __deepcopy__(self, memodict={}):

        lldg_cpy = LowLevelDependencyGraph()

        lldg_cpy.set_root(copy.deepcopy(self.get_root()))

        cpy_list = [lldg_cpy.get_root()]
        original_list =[self.get_root()]
        arg_node_map = {}
        while len(original_list) > 0:
            node = original_list.pop()
            node_cpy =cpy_list.pop()
            if node:
                children = node.get_children()
                if children is not None:
                    for child in children:
                        child_cpy = copy.deepcopy(child)
                        arg_node_map[child_cpy.arg] = child_cpy
                        node_cpy.add_child(child_cpy)
                        child_cpy.set_parent(node_cpy)
                        original_list.insert(0, child)
                        cpy_list.insert(0, child_cpy)
        lldg_cpy.arg_to_node_map = arg_node_map
        return lldg_cpy

    def __createGraph(self, chain_of_strings, arg_to_generator_type_map, depender_map, dependee_map, gen_type_to_class_map):
        chain_of_nodes = []
        position_int = 0
        for arg in chain_of_strings:
            chain_of_nodes.append(self.__createGraphNode(position_int, arg, arg_to_generator_type_map, depender_map, dependee_map, gen_type_to_class_map))
            position_int = position_int + 1
        if len(chain_of_nodes) > 0:
            prevNode = None
            for node in chain_of_nodes:
                self.add_node(node)
                if prevNode is not None:
                    node.set_parent(prevNode)
                    prevNode.set_children([node])
                prevNode = node

            self.set_root(chain_of_nodes[0])

            pairGenerator = util.pairwise(chain_of_nodes)
            try:
                a, b = pairGenerator.next()
                self.add_edge(a, b)
            except StopIteration:
                pass

    def __createGraphNode(self, position_int,  arg, arg_to_generator_type_map, depender_map, dependee_map, generator_type_to_class_map):
        node = LowLevelSequenceNode(arg)
        node.position_int = position_int
        self.arg_to_node_map[arg] = node
        generator_class_name = generator_type_to_class_map[arg_to_generator_type_map[arg]]
        generator_obj = getattr(globals()[generator_class_name], generator_class_name)()
        node.set_generator(generator_obj)
        node.arg = arg
        if depender_map:
            node.depends_on = depender_map.get(arg, [])
        if dependee_map:
            node.dependees = dependee_map.get(arg, [])
        return node

