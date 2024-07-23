#!/usr/bin/env python
import argparse
import random
import numpy as np
import time


class TMP(object):
    def __init__(self, args, policy_file=None):

        # Big hack due to the use of the Config import.
        # Need these to be imported here only when the class is initialized.
        #
        # On module load is an issue since necessary domains are not setup
        # in __main__ (main.py).
        import Config
        from src.DataStructures.PlanRefinementNode import PlanRefinementNode
        from src.DataStructures.PlanRefinementGraph import PlanRefinementGraph
        from src.PRGraphRefinementAlgorithms.PRRefinement import PRRefinement
        from src.Wrappers.ProblemSpecification import ProblemSpecification
        
        
        random.seed(int(time.time()))
        np.random.seed(int(time.time()))
        
        pddl_problem_file = Config.DEFAULT_PROBLEM_FILE
        pddl_domain_file = Config.DEFAULT_PDDL_FILE
        
        if args.problem_file is not None:
            pddl_problem_file = args.problem_file
            
        if args.domain_file is not None:
            pddl_domain_file = args.domain_file
        
        self.problem_spec = ProblemSpecification(pddl_domain_file=pddl_domain_file,
                                            pddl_problem_file=pddl_problem_file,
                                            ll_state_type=Config.LL_STATE_TYPE,
                                            hl_state_type=Config.HL_STATE_TYPE,
                                            hl_planner_name=Config.HL_PLANNER,
                                            ll_planner_name=Config.LL_PLANNER)

        self.initial_pr_node = PlanRefinementNode(problem_specification=self.problem_spec)

        self.plan_refinement_graph = PlanRefinementGraph(self.initial_pr_node)

        self.PRRef = PRRefinement(self.plan_refinement_graph,self.problem_spec,
                                  assume_refinable=args.assume_refinable,
                                  store_policy_tree=args.store_policy_tree,
                                  store_simulated_executions=args.store_simulated_executions,
                                  output_dir=args.output_dir,
                                  ll_file=args.ll_file,
                                  policy_file=policy_file,
                                  store_image=args.store_image)

        if args.set_camera:
            self.set_camera()
    
    def execute(self):
        success, policy_tree = self.PRRef.run()
        return success, policy_tree
    
    def set_plan(self, policy):
        
        self.PRRef.policy_file = policy
        
    def run_plan(self, policy_tree):
        
        self.PRRef.run_plan(policy_tree)

    def set_camera(self):

        ll_state = self.problem_spec.ll_state_type()

        assert ll_state.values is not None
        assert isinstance(ll_state.values, dict)

        assert ll_state.simulator is not None
        assert ll_state.simulator.env is not None
        viewer = ll_state.simulator.env.GetViewer()

        if "objects" in ll_state.values and "table_blue" in ll_state.values["objects"]:

            # Cafeworld camera angle setup.
            viewer.SetCamera(
                [[ -1.26668234e-02,   7.10505939e-01,  -7.03577190e-01, 8.80096436e+00],
                 [  9.99905994e-01,   5.30663706e-03,  -1.26428709e-02, -4.04841423e+00],
                 [ -5.24920604e-03,  -7.03671195e-01,  -7.10506366e-01, 7.85888958e+00],
                 [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
        elif "robots" in ll_state.values and "yumi" in ll_state.values["robots"]:

            # Keva camera angle setup.
            viewer.SetCamera(
                [[ 0.01124211,  0.48109715, -0.8765952 ,  1.10629892],
                 [ 0.99988877,  0.00318416,  0.01457086, -0.05577996],
                 [ 0.00980122, -0.8766615 , -0.48100784,  0.82077068],
                 [ 0.        ,  0.        ,  0.        ,  1.        ]])
        else:

            pass