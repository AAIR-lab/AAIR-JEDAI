import os
### FOR Hanoi world
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'


# Structure
TOWER = 'spiral_tower'
THREE_TOWER = "three_tower"
STACKED_PI='pi'

HAND = "right"
#Stochastic nature of placing planks:
STOCHASTIC=False
STRUCTURE=STACKED_PI

#Planners
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"
BACKTRACK_PROB = 1.0

if STOCHASTIC:
    TYPE='stochastic'
    HL_PLANNER = LAO_SOLVER
    DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_domain_'+STRUCTURE+'.pddl'
else:
    TYPE='deterministic'
    # HL_PLANNER = FF_PLANNER
    DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_domain.pddl'


DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_'+STRUCTURE+'.problem'
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_'+STRUCTURE+'.output'

OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/keva_double_station_two_hands.dae'

ROBOT_NAME = 'yumi'
DOMAIN_NAME = "keva"

LL_ACTION_CONFIG = DOMAIN_DIR+'ActionConfig_'+TYPE+'_two_hands.json'
REAL_ROBOT = False

IK_SOLVER = "trac_ik"
REFERENCE_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/'+STRUCTURE+'.dae'


''' FOR DEBUGGING 

# import os
# ### FOR Hanoi world
# DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'


# # Structure
# TOWER = 'spiral_tower'
# THREE_TOWER = "three_tower"
# STACKED_PI='pi'

# HAND = "right"
# #Stochastic nature of placing planks:
# STOCHASTIC=False
# STRUCTURE=STACKED_PI

# #Planners
# FF_PLANNER = 'ff'
# LAO_SOLVER = "lao"
# BACKTRACK_PROB = 1.0

# if STOCHASTIC:
#     TYPE='stochastic'
#     HL_PLANNER = LAO_SOLVER
#     DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_domain_'+STRUCTURE+'.pddl'
# else:
#     TYPE='deterministic'
#     HL_PLANNER = FF_PLANNER
#     DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_domain.pddl'

# DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/new_domain.pddl'


# DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/new_problem.pddl'
# DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_'+STRUCTURE+'.output'

# OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/option1.dae'

# ROBOT_NAME = 'yumi'
# DOMAIN_NAME = "keva"

# LL_ACTION_CONFIG = DOMAIN_DIR+'ActionConfig_'+TYPE+'_two_hands.json'
# REAL_ROBOT = False

# IK_SOLVER = "trac_ik"
# REFERENCE_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/'+STRUCTURE+'.dae'



''' 