import os
### FOR Hanoi world
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'

HAND = "left"
N_DOMINOS = 15

#Planners
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"

TYPE='stochastic'
HL_PLANNER = LAO_SOLVER
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/domain.ppddl'


DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/problem.ppddl'
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/fow.output'

#unzipping env files
# if not os.path.isfile(DOMAIN_DIR+'Environments'):
#     import tarfile
#     my_tar = tarfile.open(DOMAIN_DIR+'Environments.tar.gz')
#     my_tar.extractall(DOMAIN_DIR)
#     my_tar.close()
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/domino_env_'+str(N_DOMINOS)+'.dae'

DOMAIN_NAME = "Domino"

LL_ACTION_CONFIG = DOMAIN_DIR+'ActionConfig_Domino.json'
REAL_ROBOT = False

IK_SOLVER = "trac_ik"
RESULTS_FILE = "domain_15.csv"