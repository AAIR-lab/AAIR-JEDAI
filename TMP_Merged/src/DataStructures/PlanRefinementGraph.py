import networkx
class PlanRefinementGraph(object):
    def __init__(self,root_node = None):
        self.graph = networkx.DiGraph()
        self.graph.add_node("ROOT")
        self.root_node = root_node

    def get_root_node(self):
        return self.root_node

    def add_root_node(self, new_plan_node):
        self.graph.add_node(new_plan_node)
        self.root_node = new_plan_node


