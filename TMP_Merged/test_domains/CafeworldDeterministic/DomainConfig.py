import os
### FOR Delicate Canworld MDP
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'
  # True if need to match init position with real robot, true if executing final results on real robot.

NUM_CANS = "15"
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/domain.pddl'  # PATH to the PPDDL/PDDL domain file
DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/problem_small.pddl'  #  PATH to the PDDL/PPDDL problem file
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/problem_small.output'  # Optional : PATH to the solution file
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/env_small.dae' # Low level envrionment dae/xml path
ROBOT_NAME = 'fetch'  # NAME of the robot. Should match the robot name in the environment
DOMAIN_NAME = "cafeWorld"  # name of the domain. Optional
# LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_delicate_cans_V3.json'  # PATH to actionConfig file
LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfigPRPY.json'  # PATH to actionConfig file with prpy support
REAL_ROBOT = False  # True if need to match init position with real robot, true if executing final results on real robot.
IK_SOLVER = "ik_fast"
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
# HL_PLANNER = FF_PLANNER
LOOPED_RUNS = False
BACKTRACK_PROB = 1.0
RESULTS_FILE = "result_delicate_{}.csv".format(NUM_CANS)
