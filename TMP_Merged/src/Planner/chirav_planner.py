from src.Utils import FileUtils
import src.Utils.CommandLineUtils as CommandLineUtils
from src.Parser.TaskPlannerOutputParser import TaskPlannerOutputParser
import Config as Config
from src.Planner.Planner import Planner
from src.Parser.PDDLPlanParser import PDDLPlanParser
import networkx as nx
from src.Solution.FFSolution import FFSolution
import os

class CDPlanner(Planner):

    def __init__(self,socket):
        self.solution_file_path = os.getenv("HOME")+'/JEDAI/media/documents/solution.txt'
        self.success_str = 'found legal plan as follows'
        self.socket = socket
        pass

    def solve(self, hl_problem,problem_specification):
        # planStrFileH, rawOut, planCount = self.getResult(hl_problem.domain_file, hl_problem.problem_file,
                                                        #  hl_problem.output_file)
        # if planStrFileH == -1:
        #     return None,None

        # pddl_plan_parser = PDDLPlanParser(
        #     pddlDomainFileName=problem_specification.pddl_domain_file,
        #     pddlProblemFileName=problem_specification.pddl_problem_file,
        #     planFileName=planStrFileH)

        # list_pddl_state_objects = pddl_plan_parser.get_pddl_states()
        # print(list_pddl_state_objects)
        ffsolution, max_states = self.make_ff_solution()

        # n_states = len(list_pddl_state_objects)
        # while n_states < max_states:
        #     list_pddl_state_objects.append(list_pddl_state_objects[-1])
        #     n_states += 1

        return ffsolution,None

    def make_ff_solution(self):
        # with open(self.solution_file_path, 'r') as solution:
        #     lines = [line.strip() for line in solution.readlines() if len(line.strip()) > 0]
        # if lines:
        print "ready to recieve plan"
        self.socket.sendall("ready")
        data = str(self.socket.recv(65536))
        

        lines = data.split(",")
        lines = [line.strip() for line in lines if len(line.strip()) > 0]
        lines.append(lines[-1])
        G = nx.DiGraph()
        i = -1
        print(lines)
        for line in lines:
            print line
            if i < 0:
                root_label = "[ None :: (" + line + ") :: 1"
                G.add_node("0",label = root_label)
                print(root_label)
            else:
                label = "[ None :: (" + line +") :: 1"
                G.add_node(str(i+1),label=label)
                G.add_edge(str(i),str(i+1))
                print(label)
            i += 1
       
        ffsolution = FFSolution(True,G,"0")
        return ffsolution, len(lines)


    def __runPlanner(self, domain_file, problem_file, output_file):
        commandStr = Config.PLANNER_DIR+"FF-v2.3modified/ff" + " -o " + domain_file + " -f " + problem_file
        retVal = CommandLineUtils.executeCommand(commandStr, self.success_str, output_file)
        if retVal == -1:
            return -1, -1, -1

        rawOutput = FileUtils.read(output_file)
        ffOutStr = TaskPlannerOutputParser(rawOutput, "ff").getFFPlan()
        return ffOutStr, rawOutput, 1


    def getResult(self, domain_file, problem_file, output_file):
        planStr, rawOut, planCount = self.__runPlanner(domain_file, problem_file, output_file)
        if planStr == -1:
            return -1, -1, -1
        planStrF = FileUtils.getStringIOFile(planStr)
        print(planStr)
        return planStrF, rawOut, planCount
 
