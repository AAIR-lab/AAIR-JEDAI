import sys
import os

import config

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Problem import Problem

FOIL_COUNT = 1

preconditions_explanation = {
    "free": "Precondition {} requires gripper to be free",
    "holding": "Precondition {} requires robot to hold {}",
    "on-top": "Precondition {} requires {} to be on top of {}"
}


def find_precondition_explanation(unmet_precondition: str) -> str:
    """
    This function will give the correct explanation for the unmet precondition.

    :param unmet_precondition: unmet preccondition
    :rtype: correct explanation for the unmet precondition
    """

    # Extracting arguments from the precondition
    arguments = unmet_precondition[1:-1].split()
    print(arguments)
    if arguments[0] == "free":
        return preconditions_explanation["free"].format(unmet_precondition)
    elif arguments[0] == "holding":
        return preconditions_explanation["holding"].format(unmet_precondition, arguments[2])
    else:
        return preconditions_explanation["on-top"].format(unmet_precondition, arguments[1], arguments[2])


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
    explanation_map = problem.explain()
    return explanation_map


if __name__ == "__main__":
    call_server(sys.argv[1])
