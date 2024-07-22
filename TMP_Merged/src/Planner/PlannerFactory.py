from OpenRavePlannerV2 import OpenRavePlannerV2
from src.Planner.FFPlanner import FFPlanner
from src.Planner.LAOSolver import LAOSolver
from src.Planner.chirav_planner import CDPlanner
import Config

def create(planner_name,socket=None):
    print(planner_name)
    # if planner_name == Config.OPENRAVE_PLANNER:
    #     return OpenRavePlannerV2()
    if planner_name == Config.FF_PLANNER:
        return FFPlanner()
    elif planner_name == Config.LAO_SOLVER:
        return LAOSolver()
    elif planner_name == Config.CD_PLANNER:
        return CDPlanner(socket)

