from src.Solution.Solution import Solution
class HighLevelProblemSolution(Solution):
    def __init__(self, success=None, planStrFileH = None, resulting_state=None):
        self.success = success
        self.planStrFileH = planStrFileH
        self.state = resulting_state

    def get_final_state(self):
        return self.ll_state

    def get_failed_preconditions(self):
        pass