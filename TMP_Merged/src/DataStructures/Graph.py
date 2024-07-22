import networkx

class Graph(object):
    def __init__(self):
        self.graph = networkx.DiGraph()
        self.edges = {}
        self.__root_node = None

    def set_root(self, node):
        self.__root_node = node

    def get_root(self):
        return self.__root_node

    def add_node(self, node):
        self.graph.add_node(node)

    def add_edge(self, from_node, to_node, label=None):
        self.graph.add_edge(from_node, to_node, label=label)

    def remove_edge(self,from_node,to_node):
        self.graph.remove_edge(from_node,to_node)

    def remove_node(self,node):
        child =[node]
        while len(child) > 0:
            temp = child.pop(0)
            if temp is not None:
                children = temp.get_children()
                if children is not None:
                    for i in children:
                        child.append(i)
                self.graph.remove_node(temp)

    def add_sub_tree(self,parent,new_tree):
        # for node in new_tree.graph:
        #     self.add_node(node)
        # for edge in new_tree.graph.edges():
        #     self.add_edge(edge[0],edge[1])
        self.graph.add_nodes_from(new_tree.graph.nodes)
        self.graph.add_edges_from(new_tree.graph.edges)
        self.graph.add_edge(parent,new_tree.get_root())

    def clear_graph(self):
        self.graph.clear()


    def remove_all_edges(self):
        for edge in self.graph.edges():
            self.graph.remove_edge(edge[0],edge[1])

    def remove_all_nodes(self):
        for node in self.graph.nodes:
            self.graph.remove_node(node)