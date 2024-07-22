from Graph import Graph

class RefinedPolicy(Graph):

    def __init__(self):
        super(RefinedPolicy,self).__init__()
        self.edges = {}

    def add_node(self,node):
        super(RefinedPolicy,self).add_node(node)
        if self.get_root() is None:
            self.set_root(node)
        if node not in self.edges:
            self.edges[node] = {}

    def add_edge(self,parent,child,edge):
        super(RefinedPolicy,self).add_edge(parent,child)
        parent.add_child(child)
        child.set_parent(parent)
        self.edges[parent][child] = edge

    def get_edge(self,parent,child):
        if parent in self.edges:
            if child in self.edges[parent]:
                return self.edges[parent][child]
        return None
