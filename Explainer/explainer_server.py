import sys
import os

import config
import logging
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Problem import Problem
log = logging.getLogger(__name__)
FOIL_COUNT = 1

preconditions_explanation = {
    "free": "Precondition {} requires gripper to be free",
    "holding": "Precondition {} requires robot to hold {}",
    "on-top": "Precondition {} requires {} to be on top of {}"
}

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
VAL_INFO_COMMAND = FILE_DIR + '/scripts/val_info.sh {} {} {} ' + config.VAL_PATH
PLAN_FILE = FILE_DIR + '/temp/plan.sol'


def find_precondition_explanation(unmet_precondition: str) -> str:
    """
    This function will give the correct explanation for the unmet precondition.

    :param unmet_precondition: unmet preccondition
    :rtype: correct explanation for the unmet precondition
    """

    # Extracting arguments from the precondition
    arguments = unmet_precondition[1:-1].split()
    if arguments[0] == "free":
        return preconditions_explanation["free"].format(unmet_precondition)
    elif arguments[0] == "holding":
        return preconditions_explanation["holding"].format(unmet_precondition, arguments[2])
    else:
        return preconditions_explanation["on-top"].format(unmet_precondition, arguments[1], arguments[2])

def get_val_info():
    output = [i.strip() for i in
              os.popen(VAL_INFO_COMMAND.format(config.DOMAIN_DOCUMENT_FILE, config.PROBLEM_DOCUMENT_FILE, PLAN_FILE)).read().strip().split('\n')]
    log.debug(f"Output from VAL: {output}")
    val_info = [o.strip() for o in output[0].split("@")]
    return val_info


def generate_plan_string_from_explanation(plan, explanation_map,val_output):
    ''' 
    On precondition failure, return partial plan
    On goal failure or full success, return the entire plan as-is
    Check if HELM returns an error and then prune the output based on VAL's output
    '''

    if 'failed_precondition' in explanation_map.keys():        

        if val_output[0] != 'goal':
            plan = plan.split(',')
            badActionStep = int(val_output[1])-1
            log.debug(f"badActionStep:{badActionStep}")
            plan = (',').join(plan[:badActionStep])
            log.debug(f"plan:{plan}")
            explanation_map['badStep'] = badActionStep

    return explanation_map, plan

def call_server(plan, semantics=None):
    actions = plan.split(",")
    f = open(config.DOCUMENTS_PATH + "/foil1", "w")
    for action in actions:
        f.write('(')
        f.write(action)
        f.write(')\n')
    f.close()

    problem = Problem(
        config.DOMAIN_DOCUMENT_FILE,
        config.PROBLEM_DOCUMENT_FILE,
        config.FOIL_DOCUMENT_PATH,
        FOIL_COUNT,
        config.LATTICE_DOCUMENT_FILE,
        config.DOMAIN_TEMPL_DOCUMENT_FILE,
        config.PROBLEM_TEMPL_DOCUMENT_FILE,
        semantics
    )
    explanation_map, exec_plan = generate_plan_string_from_explanation(plan=plan, explanation_map=problem.explain(), val_output=get_val_info())
    return explanation_map, exec_plan


if __name__ == "__main__":
    call_server(sys.argv[1])
