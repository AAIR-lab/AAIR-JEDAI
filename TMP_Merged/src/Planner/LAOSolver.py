
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

    def __init__(self):
        self.succ_str = "Found legal sequence actions"
        self.success = True
        if not os.path.isfile(Config.PLANNER_DIR+'mdp-lib'):
            import tarfile
            my_tar = tarfile.open(Config.PLANNER_DIR+'mdp-lib.tar.gz')
            my_tar.extractall(Config.PLANNER_DIR)
            my_tar.close()

    def solve(self,hlproblem,problem_specification):
        self.__runPlanner(domain_file=hlproblem.domain_file,problem_file=hlproblem.problem_file)
        graph,root = self.prepare_planStr()
        if not self.success:
            return None,None
        f = open(Config.POLICY_OUTPUT_FILE,"r")
        lines = f.readlines()
        cost_line = lines[-1]
        cost = float(cost_line.split("Cost:")[-1])
        laosolution = LAOSolution(self.succ_str,graph,root,cost)
        return laosolution,None

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
                if time.time() - start_time > 200:
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

    def prepare_planStr(self):
        graph = nx.drawing.nx_pydot.read_dot(Config.POLICY_OUTPUT_FILE)
        # os.remove(os.curdir+"graph.gv")
        root = None
        for node in graph.nodes():
            if graph.in_degree[node] ==0:
                root = node
                break
        return graph,root