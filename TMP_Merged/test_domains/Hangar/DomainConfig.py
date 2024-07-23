import os
### FOR Delicate Canworld MDP
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'

# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/'
#
# ROBOT_NAME = 
# 
# LL_ACTION_CONFIG = PROJ_DIR+
# UAV_XML = PROJ_DIR + "GeneratedEnvironments/Hanger/test_dummy.robot.xml"
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/Hanger/hanger_world.dae'
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/hanger_5_domain.pddl'  # PATH to the PPDDL/PDDL domain file
DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/hanger_problem.pddl'  #  PATH to the PDDL/PPDDL problem file
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/hanger_problem.output'  # Optional : PATH to the solution file
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/can_world_'+NUM_CANS+'_cans.dae' # Low level envrionment dae/xml path
ROBOT_NAME = "UAV"  # NAME of the robot. Should match the robot name in the environment
DOMAIN_NAME = "hanger"  # name of the domain. Optional
# LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_delicate_cans_V3.json'  # PATH to actionConfig file
LL_ACTION_CONFIG = DOMAIN_DIR + 'ActionConfig_HangerV3.json'  # PATH to actionConfig file with prpy support
REAL_ROBOT = False  # True if need to match init position with real robot, true if executing final results on real robot.
IK_SOLVER = "ik_fast"
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
HL_PLANNER = LAO_SOLVER