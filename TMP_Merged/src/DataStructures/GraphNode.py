class GraphNode(object):
    def __init__(self):
        self.visited = False
        self.goal = False
        self.parent = None

    def add_child(self, child):
        raise NotImplementedError("add_child() has to be implemented by subclass of Graph")

    def set_children(self, children):
        raise NotImplementedError("set_children() has to be implemented by subclass of Graph")

    def get_parent(self):
        raise NotImplementedError("get_parent() has to be implemented by subclass of GraphNode")

    def get_children(self):
        raise NotImplementedError("get_children() has to be implemented by subclass of Graph")
