from src.Solution.Solution import Solution
import copy

class FFSolution(Solution):

    def __init__(self,success = None,policyTree = None,root = None):
        self.success = success
        self.policyTree = policyTree
        self.root = root

    def get_root_node(self):
        return self.root


    def __deepcopy__(self, memodict={}):
        success_copy = copy.deepcopy(self.success)
        policTree_copy = copy.deepcopy(self.policyTree)
        root_copy = copy.deepcopy(self.root)

        ff_solution_copy = FFSolution(success_copy,policTree_copy,root_copy)
        return ff_solution_copy