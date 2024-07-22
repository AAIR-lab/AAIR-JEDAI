from src.Utils import FileUtils
import src.Utils.CommandLineUtils as CommandLineUtils
from src.Parser.TaskPlannerOutputParser import TaskPlannerOutputParser
import Config as Config
from src.Planner.Planner import Planner
from src.Parser.PDDLPlanParser import PDDLPlanParser
import networkx as nx
from src.Solution.FFSolution import FFSolution

class FFPlanner(Planner):

    def __init__(self):
        self.success_str = 'found legal plan as follows'
        pass

    def solve(self, hl_problem,problem_specification):
        planStrFileH, rawOut, planCount = self.getResult(hl_problem.domain_file, hl_problem.problem_file,
                                                         hl_problem.output_file)
        if planStrFileH == -1:
            return None,None

        pddl_plan_parser = PDDLPlanParser(
            pddlDomainFileName=hl_problem.domain_file,
            pddlProblemFileName=hl_problem.problem_file,
            planFileName=planStrFileH)

        list_pddl_state_objects = pddl_plan_parser.get_pddl_states()

        ffsolution = self.make_ff_solution(planStrFileH,rawOut)


        return ffsolution,list_pddl_state_objects

    def make_ff_solution(self,planStrFileH,rawOut):
        planStrFileH.seek(0)
        actions = planStrFileH.read().split("\n")[2:-3]
        G = nx.DiGraph()
        print(planStrFileH.read())
        root_label = "[ None :: (" + actions[0].split(":")[1].strip().lower() + ") :: 1"
        G.add_node("0",label = root_label)
        i = 0
        for action in actions[1:]:
            label = "[ None :: (" + action.split(":")[1].strip().lower() + ") :: 1"
            G.add_node(str(i+1),label=label)
            G.add_edge(str(i),str(i+1))
            i+=1
        last_node_label = "[ None :: (done) :: 1"
        G.add_node(str(i+1),label=last_node_label)
        G.add_edge(str(i),str(i+1))
        ffsolution = FFSolution(True,G,"0")
        return ffsolution



    def __runPlanner(self, domain_file, problem_file, output_file):
        commandStr = Config.PLANNER_DIR+"FF-v2.3modified/ff" + " -o " + domain_file + " -f " + problem_file

        retVal = CommandLineUtils.executeCommand(commandStr, self.success_str, output_file)
        if retVal == -1:
            return -1, -1, -1

        rawOutput = FileUtils.read(output_file)
        ffOutStr = TaskPlannerOutputParser(rawOutput, "ff").getFFPlan()
        print(ffOutStr)
        return ffOutStr, rawOutput, 1


    def getResult(self, domain_file, problem_file, output_file):
        planStr, rawOut, planCount = self.__runPlanner(domain_file, problem_file, output_file)
        if planStr == -1:
            return -1, -1, -1
        planStrF = FileUtils.getStringIOFile(planStr)
        return planStrF, rawOut, planCount