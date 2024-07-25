from src.Solution.Solution import Solution


class LAOSolution(Solution):

    def __init__(self,success = None, policyTree = None , root = None,cost = 0 ):
        self.success = success
        self.policyTree =policyTree
        self.root = root
        self.cost = cost

    def get_root_node(self):
        return self.root
