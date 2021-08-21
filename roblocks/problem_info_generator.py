from pddlpy import DomainProblem

import config
from natural_language import natural_language_generator as nlg


# TODO goal display
def get_problem_info(semantics):
    domprob = DomainProblem(config.DOMAIN_DOCUMENT_FILE, config.PROBLEM_DOCUMENT_FILE)

    goals = domprob.goals()
    goal_strs = []
    for goal in goals:
        goal_strs.append(str(goal).replace("'", "").replace(",", ""))
    print("goal_strs before nlg:", goal_strs)
    # TODO assuming all goals are positive because negative goals are not parsed correctly
    goal_str = nlg.get_natural_language_multiple_predicates(semantics, goal_strs, [False for _ in goal_strs])
    print("goal_str after nlg:", goal_str)

    operators = list(domprob.domain.operators)
    actions = []
    for operator in operators:
        action = {"name": operator}
        params = []
        for p, t in domprob.domain.operators[operator].variable_list.items():
            param = {"name": p, "type": t}
            params.append(param)
        action["params"] = params
        actions.append(action)
    objects = []
    for operator, t in domprob.problem.objects.items():
        obj = {"name": operator, "type": t}
        objects.append(obj)
    planning_elements = {"Actions": actions, "Objects": objects}

    return {"goal_str": goal_str, "planning_elements": planning_elements}
