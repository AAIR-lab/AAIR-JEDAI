from src.DataStructures.GraphNode import GraphNode
import copy

class RefinedPolicyNode(GraphNode):

    def __init__(self,hl_state):
        super(RefinedPolicyNode,self).__init__()
        self.children = []
        self.hl_state = hl_state
        self.ll_state = None
        self.is_goal = False


    def add_child(self, child):
        self.children.append(child)

    def set_children(self, children):
        self.children = copy.deepcopy(children)

    def get_parent(self):
        return self.parent

    def get_children(self):
        return self.children

    def set_parent(self,parent):
        self.parent = parent