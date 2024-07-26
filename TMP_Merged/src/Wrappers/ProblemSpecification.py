import src.States as States
from src.States import *


class ProblemSpecification(object):
    def __init__(self, pddl_domain_file =None, pddl_problem_file=None, ll_state_type=None, hl_state_type = None, hl_planner_name=None, ll_planner_name=None):
        self.pddl_domain_file = pddl_domain_file
        self.pddl_problem_file = pddl_problem_file
        self.hl_state_type = hl_state_type
        self.ll_state_type = ll_state_type
        self.hl_planner_name = hl_planner_name
        self.ll_planner_name = ll_planner_name

        if type(hl_state_type) == str:
            self.hl_state_type = getattr(getattr(States, hl_state_type), hl_state_type)

        if type(ll_state_type) == str:
            self.ll_state_type = getattr(globals()[ll_state_type], ll_state_type)
