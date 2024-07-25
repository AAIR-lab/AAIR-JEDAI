import sys,os

PROJ_DIR = os.path.abspath(os.path.dirname(__file__))+'/'
HOME_DIR = os.path.expanduser("~") + '/'

MAX_TIME = 1200

K = 3
DEBUG = False
REAL_ROBOT = False

### FOR Canworld


# NUM_CANS = "2"
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/robotics_fetch_canworld_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_problem.output'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "can_world"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_canworldV3.json'
# ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_rot"]
# FETCH_URDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
# FETCH_SRDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.srdf'
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'


### FOR Canworld MDP

# NUM_CANS = "20"
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/new_canworld_mdp_'+NUM_CANS+'_cans_5_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_mdp_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/new_canworld_'+NUM_CANS+'_cans_problem.output'
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "can_world"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_canworldV3.json'
# ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_rot"]
# FETCH_URDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
# FETCH_SRDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.srdf'

### FOR Delicate Canworld MDP
#
NUM_CANS = "20"
DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+NUM_CANS+'_cans_10_mdp.pddl'
DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+NUM_CANS+'_cans_problem.pddl'
DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/'+NUM_CANS+'_can_world_delicate_cans.output'
OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'
# OPENRAVE_ENV_XML = PROJ_DIR + "GeneratedEnvironments/real_can_world_10_cans.dae"
ROBOT_NAME = 'fetch'
DOMAIN_NAME = "delicate_canworld"
LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_delicate_cans_V3.json'
ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_rot"]
FETCH_URDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
FETCH_SRDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.srdf'
POLICY_OUTPUT_FILE = PROJ_DIR + "graph1.gv"
COMBINED_FILE = PROJ_DIR + "combined_file0.pddl"

### FOR Delicate Canworld MDP Physical Execution
#
# NUM_CANS = "15"
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+NUM_CANS+'_cans_10_mdp.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/can_world_delicate_cans_'+NUM_CANS+'_cans_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/'+NUM_CANS+'_can_world_delicate_cans.output'
# OPENRAVE_ENV_XML = PROJ_DIR + "GeneratedEnvironments/real_can_world_10_cans.dae"
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "delicate_canworld"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_real_delicate_cans_V3.json'
# ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_rot"]
# FETCH_URDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
# FETCH_SRDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.srdf'
# POLICY_OUTPUT_FILE = PROJ_DIR + "graph1.gv"
# COMBINED_FILE = PROJ_DIR + "combined_file0.pddl"
# REAL_ROBOT = True

### FOR Hanoi world

# NUM_BOXES = '5'
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/hanoi_world_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/hanoi_world_'+NUM_BOXES+'_boxes_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/robotics_fetch_hanoi_'+NUM_BOXES+'_boxes_problem.output'
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/hanoi_world_'+NUM_BOXES+'_boxes.dae'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "hanoi_world"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfig_hanoiV3.json'
# ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_rot"]
# FETCH_URDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
# FETCH_SRDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.srdf'


### FOR Canworld2

# NUM_CANS = '3'
# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/canworld2_mdp_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/new_canworld2_3_cans_mdp_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/mdp_robotics_fetch_'+NUM_CANS+'_cans_problem.output'
#
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/can_world_'+NUM_CANS+'_cans.dae'
# ROBOT_NAME = 'fetch'
# DOMAIN_NAME = "can_world2"
# LL_ACTION_CONFIG = PROJ_DIR + 'ActionConfigV2_canworld2.json'
# ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_rot"]
# FETCH_URDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
# FETCH_SRDF = PROJ_DIR + 'RobotModels/fetch/URDF/fetch.srdf'


#
#



### FOR Hangar World

# DEFAULT_PDDL_FILE = PROJ_DIR+'SampleTasks/hanger_5_domain.pddl'
# DEFAULT_PROBLEM_FILE = PROJ_DIR+'SampleTasks/hanger_problem.pddl'
# DEFAULT_OUTPUT_FILE = PROJ_DIR+'SampleTasks/hanger_problem.output'
#
# ROBOT_NAME = "UAV"
# DOMAIN_NAME = "hanger"
# LL_ACTION_CONFIG = PROJ_DIR+'ActionConfig_HangerV3.json'
# UAV_XML = PROJ_DIR + "GeneratedEnvironments/Hanger/test_dummy.robot.xml"
# OPENRAVE_ENV_XML = PROJ_DIR+'GeneratedEnvironments/Hanger/hanger_world.dae'
# ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_z"]
# POLICY_OUTPUT_FILE = PROJ_DIR + "graph1.gv"
# COMBINED_FILE = PROJ_DIR + "combined_file0.pddl"



### General Arguments

SHOW_VIEWER = True
NO_MOTION_PLAN_CHECK = False
REUSE_OPENRAVE_ENV = True

LOG_LEVEL = 'DEBUG'




FastForwardExe = PROJ_DIR+"TaskPlanners/FF-v2.3modified/ff"
PDDL_STATE = 'pddl_state'



IK_SOLVER = "ik_fast"
# IK_SOLVER = "trac_ik"

EXECUTE_MOTION_PLANS = False
MAX_IKs_TO_CHECK_FOR_MP = 20

OPENRAVE_PLANNER = 'openrave_planner'
OPENRAVE_NATIVE_MOTION_PLANNER = 'OpenRave_NativeMotion_Planner'
# MOTION_PLANNER = OPENRAVE_NATIVE_MOTION_PLANNER
MOTION_PLANNER = "OMPL_RRTConnect"
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"

OPENRAVE_LL_ENV = "OpenRaveLowLevelState"

HL_STATE_TYPE = "PDDLState"

LL_STATE_TYPE = OPENRAVE_LL_ENV

#HL_PLANNER = FF_PLANNER
HL_PLANNER = LAO_SOLVER

HORIZON = 8

RUN_TRAJ = True


LL_PLANNER = OPENRAVE_PLANNER


LL_ENV = OPENRAVE_LL_ENV

LL_PLANNER_ARGS = {"world":"aair_lab", "robot":"fetch", "simulator":"openrave","doMapRobotJoints":False}

OPENRAVE_CREATED_ENV_SAVE_FILE = '/tmp/hanoiworld_env.dae'

PLOT = False
PORTNO = 1234
RESULTS_FILE = "results_20_10.csv"

VIEWER = 'qtcoin'

BACKTRACK_PROB = 0.0








PR_STRATEGY = "cost_batch_dfs"

def blockprint():
    pass
    return
    # sys.stdout = open(os.devnull,"w")

def enablePrint():
    pass
    return
    # sys.stdout = sys.__stdout__

def setPaths():
    # Adds correct path in URDF files
    sedstr = "sed -i \"s|project_directory|"+PROJ_DIR[:-1]+"|g\" "+PROJ_DIR
    os.system(sedstr +"GeneratedEnvironments/Hanger/UAV.urdf")
    os.system(sedstr +"RobotModels/fetch/URDF/fetch.urdf")
    os.system(sedstr +"RobotModels/yumi_urdf/yumi_description/urdf/yumi.urdf")
 
def resetPaths():
    sedstr = "sed -i \"s|"+PROJ_DIR[:-1]+"|project_directory|g\" "+PROJ_DIR
    os.system(sedstr +"GeneratedEnvironments/Hanger/UAV.urdf")
    os.system(sedstr +"RobotModels/fetch/URDF/fetch.urdf")
    os.system(sedstr +"RobotModels/yumi_urdf/yumi_description/urdf/yumi.urdf")
