from src.Solution.Solution import Solution
class LowLevelProblemSolution(Solution):
    def __init__(self, success, list_ll_plan, resulting_ll_state):
        self.success = success
        self.list_ll_plan = list_ll_plan
        self.state = resulting_ll_state
