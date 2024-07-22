import sys,os
################################# System paths ##################################
PROJ_DIR = os.path.abspath(os.path.dirname(__file__))+'/'
HOME_DIR = os.path.expanduser("~") + '/'
TEST_DIR = PROJ_DIR + "test_domains/"
MISC_DIR = PROJ_DIR + "misc/"
PLANNER_DIR = PROJ_DIR + "planners/"

################################# Domain Selection ##################################
# DOMAIN = 'DelicateCan'
# DOMAIN = 'DelicateCanDeterministic'
# DOMAIN = 'FetchCanPhysicalWorld'
# DOMAIN = 'HanoiDeterministic'
# DOMAIN = 'KevaDeterministic'
# DOMAIN = 'Manufacture'
# DOMAIN = 'KevaLooped' 
# DOMAIN = 'TorsenLSD'
# Domain = 'Hangar'
# DOMAIN = "Factory"
import __builtin__
if hasattr(__builtin__, "DOMAIN"):
    DOMAIN = __builtin__.DOMAIN
else:
    DOMAIN = "DominoDeterministic"

# DOMAIN = "KevaDeterministic"



# This is not actually a domain but a placeholder name used for testing. should not be uncommented.
# DOMAIN = 'Testing'

################################# General Arguments ##################################

DEBUG = False
SHOW_VIEWER = True  # False if do not want to open the viewer.
RUN_TRAJ = True # To enable final reuslts simulations
STORE_REFINED = False  # true if final refined policy needs to be pickled.
PLOT = False # To plot the refinement profile.
LOOPED_RUNS = False

# PRGraph refinement strategies options: "cost_batch_dfs" / "dfs"
PR_STRATEGY = "dfs"
ANYTIME = False
NACTIONS = 2


VIEWER = 'qtcoin'  # name of the viewer.

MAX_TIME = 1000000000
K = 4
HORIZON = 5
# Probability for stochastic choice between updating the model or backtracking.
BACKTRACK_PROB = 0.0

# Planners
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
CD_PLANNER = "cd"
PC = False
# Selection of high-level planner ( FF_PLANNER for PDDL/ LAO_SOLVER for PPDDL)
HL_PLANNER = CD_PLANNER
# HL_PLANNER = LAO_SOLVER

# Options : "WARNING"/"DEBUG"
LOG_LEVEL = 'DEBUG'

OPENRAVE_LL_ENV = "OpenRaveLowLevelState"

PDDL_STATE = 'pddl_state'
HL_STATE_TYPE = "PDDLState"
LL_STATE_TYPE = OPENRAVE_LL_ENV
# LL_STATE_TYPE = "PDDLLowLevelState"

COLLISION_CHECKER = 'pqp'

# IK solvers.
IK_SOLVER = "ik_fast"
# IK_SOLVER = "trac_ik"

# maximum number of iks to check for motion planning
MAX_IKs_TO_CHECK_FOR_MP = 20

# Selection of which motion planner to use for motion planning
# if generating motion plan through OpenRaveSimulator.
OPENRAVE_NATIVE_MOTION_PLANNER = 'OpenRave_NativeMotion_Planner'

# MOTION_PLANNER = OPENRAVE_NATIVE_MOTION_PLANNER
MOTION_PLANNER = "OMPL_RRTConnect"

LL_PLANNER = MOTION_PLANNER
################################# Domain Specific Configs ##################################

# These can override all General Arguments
sys.path.append(TEST_DIR+DOMAIN+'/')
print(TEST_DIR+DOMAIN+'/')
# Config file specific to Domain. Importing here to override general args.

from DomainConfig import *
# Policy path, needed if using PPDDL
POLICY_OUTPUT_FILE = DOMAIN_DIR + "Tasks/graph1.gv"
# Temp file path, needed if using PPDDL
COMBINED_FILE = DOMAIN_DIR + "Tasks/combined_file0.pddl"
# name of the file to store final results. used if PLOT is true.

################################# Robot Specific Configs ##################################

YUMI_URDF = MISC_DIR + 'RobotModels/yumi_urdf/yumi_description/urdf/yumi.urdf'
YUMI_SRDF = MISC_DIR + 'RobotModels/yumi_urdf/yumi_description/urdf/yumi.srdf'

FETCH_URDF = MISC_DIR + 'RobotModels/fetch/URDF/fetch.urdf'
FETCH_SRDF = MISC_DIR + 'RobotModels/fetch/URDF/fetch.srdf'
# ROBOT_NAME is retried from DomainConfig import

# if ROBOT_NAME == 'fetch':
#     ROBOT_URDF = FETCH_URDF
#     ROBOT_SRDF = FETCH_SRDF
#     ROBOT_MANIP_DOF = 8
# elif ROBOT_NAME == 'yumi':
#     ROBOT_URDF = YUMI_URDF
#     ROBOT_SRDF = YUMI_SRDF
#     ROBOT_MANIP_DOF = 7
# elif ROBOT_NAME == 'UAV':
#     ROBOT_BASE_JOINTS = ["dummy_x","dummy_y","dummy_z"]
