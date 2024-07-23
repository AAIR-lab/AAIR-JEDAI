import os
### FOR Delicate Canworld MDP
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'
  # True if need to match init position with real robot, true if executing final results on real robot.

NUM_CANS = "15"
PROB = "10"
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/can_world_delicate_cans_'+NUM_CANS+'_cans_'+PROB+'_mdp.pddl'  # PATH to the PPDDL/PDDL domain file
DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/can_world_delicate_cans_'+NUM_CANS+'_cans_problem.pddl'  #  PATH to the PDDL/PPDDL problem file
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/'+NUM_CANS+'_can_world_delicate_cans.output'  # Optional : PATH to the solution file
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/can_world_'+NUM_CANS+'_cans.dae' # Low level envrionment dae/xml path
ROBOT_NAME = 'fetch'  # NAME of the robot. Should match the robot name in the environment
DOMAIN_NAME = "delicate_canworld"  # name of the domain. Optional
# LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_delicate_cans_V3.json'  # PATH to actionConfig file
LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_delicate_cans_V3PRPY.json'  # PATH to actionConfig file with prpy support
REAL_ROBOT = False  # True if need to match init position with real robot, true if executing final results on real robot.
IK_SOLVER = "ik_fast"
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
HL_PLANNER = LAO_SOLVER
LOOPED_RUNS = False
RESULTS_FILE = "result_delicate_15_10.csv"