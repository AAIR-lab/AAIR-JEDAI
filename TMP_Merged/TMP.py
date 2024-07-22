#!/usr/bin/env python
import sys
import __builtin__
__builtin__.DOMAIN=sys.argv[1]
problem = sys.argv[2]
# __builtin__.DOMAIN="KevaDeterministic"
# __builtin__.DOMAIN="HanoiDeterministic"
import Config
import os
file_dir=os.path.dirname(os.path.abspath(__file__))[:-10]
Config.DEFAULT_PDDL_FILE=file_dir+"media/documents/domainD.pddl"
Config.DEFAULT_PROBLEM_FILE=file_dir+"media/documents/problemP.pddl"
Config.OPENRAVE_ENV_XML=file_dir+"media/documents/env.dae"
Config.REFERENCE_STRUCTURE_PATH = Config.DOMAIN_DIR + 'Environments/'+ problem +'.dae'


import argparse
from src.DataStructures.PlanRefinementNode import PlanRefinementNode
from src.DataStructures.PlanRefinementGraph import PlanRefinementGraph
from src.PRGraphRefinementAlgorithms.JEDAI_PRRefinement import PRRefinement
from src.Wrappers.ProblemSpecification import ProblemSpecification
import src.util as util

import random
import numpy as np
import time

class TMP(object):
    def __init__(self, args=None):
        random.seed(int(time.time()))
        np.random.seed(int(time.time()))
        self.problem_spec = ProblemSpecification(pddl_domain_file=Config.DEFAULT_PDDL_FILE,
                                            pddl_problem_file=Config.DEFAULT_PROBLEM_FILE,
                                            ll_state_type=Config.LL_STATE_TYPE,
                                            hl_state_type=Config.HL_STATE_TYPE,
                                            hl_planner_name=Config.HL_PLANNER,
                                            ll_planner_name=Config.LL_PLANNER)

        self.initial_pr_node = PlanRefinementNode(problem_specification=self.problem_spec)

        self.plan_refinement_graph = PlanRefinementGraph(self.initial_pr_node)

        self.PRRef = PRRefinement(self.plan_refinement_graph,self.problem_spec)
    
    def execute(self):
        success = self.PRRef.run()
        return success

if __name__ == "__main__":

    # parser = argparse.ArgumentParser()
    # parser.add_argument('--domain', type=str, help='PDDL Domain File', default=Config.DEFAULT_PDDL_FILE)
    # parser.add_argument('--problem', type=str, help='PDDL problem File', default=Config.DEFAULT_PROBLEM_FILE)
    # parser.add_argument('--lldomain', type=str, help='Low Level PDDL or Simulator environment XML')
    # parser.add_argument('--plannerName', type=str, help='High Level Planner', default='ff')
    # parser.add_argument('--outputFile', type=str, help='Plan Output File', default=Config.DEFAULT_OUTPUT_FILE)
    # args = parser.parse_args()
    start_time = time.time()
    print(start_time)
    util.blockprint()
    tmp = TMP()
    tmp.execute()
    util.enablePrint()
    print("--- %s seconds ---" % (time.time() - start_time))
    # raw_input("Exit?")
    exit(0)
