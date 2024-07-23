import os
### FOR Hanoi world
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'

HAND = "left"

#Planners
FF_PLANNER = 'ff'

TYPE='deterministic'
HL_PLANNER = FF_PLANNER
DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/fow_domain.pddl'


DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/fow_problem.pddl'
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/fow.output'

OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/two_tables_parts.dae'

ROBOT_NAME = 'yumi'
DOMAIN_NAME = "TorsenLSD"

LL_ACTION_CONFIG = DOMAIN_DIR+'ActionConfig_Fow.json'
REAL_ROBOT = True

IK_SOLVER = "trac_ik"

LEFT_REFERENCE_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/reference/lefthalf.dae'
RIGHT_REFERENCE_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/reference/righthalf.dae'
FETCH_REFERENCE_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/reference/fetch_putdown.dae'
WHEELS_REFERENCE_PATH = DOMAIN_DIR + 'Environments/reference/wheels_putdown.dae'

LEFT_ASSEMBLED_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/reference/lefthalf_assembled.dae'
RIGHT_ASSEMBLED_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/reference/righthalf_assembled.dae'
FULL_ASSEMBLED_STRUCTURE_PATH = DOMAIN_DIR + 'Environments/reference/full_assembly.dae'

MANUAL_PART_POSITIONS_PKL = DOMAIN_DIR + 'Environments/manual_part_positions.pkl'
INIT_TRANSFORMS_PKL = DOMAIN_DIR + 'Environments/init_transforms.pkl'

body_name_dict = {
    'base' : 'Base',
    # Left Half
    'l_spur_gear_2' : 'LeftWormSpur2',
    'l_spur_gear_1' : 'LeftWormSpur1',
    'l_spur_gear_3' : 'LeftWormSpur3',
    'l_ring_gear' : 'LeftRingGear',
    'l_upright' : 'LeftUprightClamp',
    'l_housing' : 'LeftHousing',
    'l_worm_gear' : 'LeftDrive',
    'l_wheel' : 'LeftWheel',  
    'l_shaft_1' : 'LeftShaft1',
    'l_shaft_2' : 'LeftShaft2',
    'l_shaft_3' : 'LeftShaft3',
    'l_axle' : 'LeftAxle',
    # Right Half
    'r_spur_gear_1' : 'RightWormSpur1',
    'r_spur_gear_2' : 'RightWormSpur2',
    'r_spur_gear_3' : 'RightWormSpur3',
    'r_upright' : 'RightUprightClamp',
    'r_housing' : 'RightHousing',
    'r_worm_gear' : 'RightDrive',
    'r_wheel' : 'RightWheel',
    'r_shaft_1' : 'RightShaft1',
    'r_shaft_2' : 'RightShaft2',
    'r_shaft_3' : 'RightShaft3',
    'r_axle' : 'RightAxle',
}
