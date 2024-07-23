
from src.Planner.Planner import Planner
import src.Utils.CommandLineUtils as CommandLineUtils
from src.Solution.LAOSolution import LAOSolution
import networkx as nx
import  threading
import os
import multiprocessing
import Config
import subprocess
import time


class LAOSolver(Planner):

    def __init__(self, policy_file=None):
        self.succ_str = "Found legal sequence actions"
        self.success = True
        self.policy_file = policy_file
        # if not os.path.isfile(Config.PLANNER_DIR+'mdp-lib'):
        #     import tarfile
        #     my_tar = tarfile.open(Config.PLANNER_DIR+'mdp-lib.tar.gz')
        #     my_tar.extractall(Config.PLANNER_DIR)
        #     my_tar.close()

    def solve(self, hlproblem, problem_specification, raise_exception=True):
        
        if self.policy_file is None:
            self.policy_file = Config.POLICY_OUTPUT_FILE
            self.__runPlanner(domain_file=hlproblem.domain_file,problem_file=hlproblem.problem_file)
            
            if not self.success:
                return None,None
        
        try:
            graph, root, cost = self.prepare_planStr(self.policy_file)

            laosolution = LAOSolution(self.succ_str,graph,root,cost)
            return laosolution,None
        except Exception as e:
            
            if raise_exception:
                raise e
            
            return None, None
        

    def create_domainproblem_combined_file(self, domain_file, problem_file):
        dom_prb_file_path = Config.COMBINED_FILE
        with open(domain_file) as dom_file:
            with open(problem_file) as pro_file:
                with open(dom_prb_file_path, "w") as comb_file:
                    comb_file.write(dom_file.read())
                    comb_file.write(pro_file.read())
        return dom_prb_file_path

    def __runPlanner(self,domain_file="canworld_mdp.pddl",problem_file = "p01",output_file = None):
        # if "graph.gv" in os.listdir(os.curdir):
        #     os.remove("graph.gv")
        dom_prob_file_path = self.create_domainproblem_combined_file(domain_file,problem_file)
        while True:
            killed = False
            p1 = subprocess.Popen([Config.PLANNER_DIR+"mdp-lib/testppddl.out",dom_prob_file_path,"p01",str(Config.HORIZON),Config.POLICY_OUTPUT_FILE])
            start_time = time.time()
            while p1.poll() is None:
                if time.time() - start_time > 60:
                    killed = True
                    self.success = False
                    p1.kill()
                    time.sleep(1)
                    break
            codep1 = p1.wait()
            if codep1 == 0  or codep1 == -6 or killed:
                if not killed:
                    self.success = True
                break
            else:
                self.success = False
                break

    def prepare_planStr(self, policy_file):
        
        cost = 0.0
        if isinstance(policy_file, str):
            graph = nx.drawing.nx_pydot.read_dot(policy_file)
            
            f = open(policy_file,"r")
            lines = f.readlines()
            cost_line = lines[-1]
            cost = float(cost_line.split("Cost:")[-1])
        else:
            graph = policy_file
        # os.remove(os.curdir+"graph.gv")
        
        # Need to set the goal attribute for places where the goal
        # is reached.
        for u, v, edge_key in graph.edges:
            
            edge_data = graph.get_edge_data(u, v, edge_key)
            if "color" in edge_data:
                
                label = graph.nodes[v]["label"]
                assert ":: STOP ::" in label
                label = label.replace(":: STOP ::", ":: tmp_goal_reached ::")
                graph.nodes[v]["label"] = label
                pass
        
        root = None
        for node in graph.nodes():
            if graph.in_degree[node] ==0:
                root = node
                break
        return graph, root, cost