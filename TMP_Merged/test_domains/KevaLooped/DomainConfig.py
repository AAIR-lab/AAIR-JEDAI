import os
### FOR Hanoi world
DOMAIN_DIR = os.path.abspath(os.path.dirname(__file__))+'/'


# Structure
TOWER='tower_4'
PI='pi'
HANDS='_two_hands'
# HANDS=''
#Stochastic nature of placing planks:
STOCHASTIC=True
STRUCTURE=PI

LOOPED_RUNS=True
NUMBER_OF_RUNS = 3
PLANKS_PER_LOOP = 4

#Planners
FF_PLANNER = 'ff'
LAO_SOLVER = "lao"

if STOCHASTIC:
    TYPE='stochastic'
    HL_PLANNER = LAO_SOLVER
    DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_domain_'+STRUCTURE+'.pddl'
else:
    TYPE='deterministic'
    HL_PLANNER = FF_PLANNER
    DEFAULT_PDDL_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_domain.pddl'


DEFAULT_PROBLEM_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_'+STRUCTURE+'.problem'
DEFAULT_OUTPUT_FILE = DOMAIN_DIR+'Tasks/keva_'+TYPE+'_'+STRUCTURE+'.output'

ROBOT_NAME = 'yumi'
DOMAIN_NAME = "keva"

LL_ACTION_CONFIG = DOMAIN_DIR+'ActionConfig_'+TYPE+HANDS+'.json'
REAL_ROBOT = False

if STRUCTURE == TOWER:
    R_STRUCT = 'Environments/NEW/keva_24_plank_spiral_tower_structure.dae'
    # R_STRUCT = 'Environments/NEW/keva_three_tower.dae'
elif STRUCTURE == PI:
    R_STRUCT = 'Environments/NEW/keva_tilted_pillar_triple_pi_structure.dae'
REFERENCE_STRUCTURE_PATH = DOMAIN_DIR + R_STRUCT

RESULTS_FILE = "results_keva_"+STRUCTURE+"_no_pc.csv"

def get_run_number():
    try:
        f = open(DOMAIN_DIR+'run_count.txt',"r")
    except IOError:
        with open(DOMAIN_DIR+'run_count.txt',"w") as f:
            f.write("1")
            f.close()
            return 1
    else:
        run_num = f.read()
        run_num = int(run_num)
        f.close()
        return run_num
OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/keva_double_run_'+str(get_run_number())+'.dae'
# OPENRAVE_ENV_XML = DOMAIN_DIR+'Environments/NEW/keva_double_station_final.dae'
def correct_plank_name(name):
    run_num = get_run_number()  
    name = "plank"+ str(int(name.split('plank')[1])+PLANKS_PER_LOOP*(run_num-1))
    print(name)
    return name
