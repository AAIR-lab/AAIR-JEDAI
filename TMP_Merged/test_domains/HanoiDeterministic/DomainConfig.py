import os
### FOR Hanoi world
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'

NUM_BOXES = '3'
BACKTRACK_PROB = 1.0
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/hanoi_world_domain.pddl'
DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/hanoi_world_'+NUM_BOXES+'_boxes_problem.pddl'
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/robotics_fetch_hanoi_'+NUM_BOXES+'_boxes_problem.output'
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/hanoi_world_'+NUM_BOXES+'_boxes.dae'
ROBOT_NAME = 'fetch'
DOMAIN_NAME = "hanoi_world"
LL_ACTION_CONFIG = DOMAIN_DIR+'ActionConfig_hanoiV3.json'
REAL_ROBOT = False