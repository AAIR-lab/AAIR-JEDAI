from tarski.io import PDDLReader
from tarski.syntax import Atom
import logging

import config
from natural_language import natural_language_generator as nlg

log = logging.getLogger(__name__)


def get_problem_info(semantics):
    """
    Uses the domain and problem files in media/documents/ to return some info about the problem in the following format:
    {
        "goal_str": <natural language string representing goal>,
        "actions": [
            {
                "name": <action name>
                "params": [
                    {
                        "name": <parameter name>,
                        "type": <parameter type name>
                    },
                    ...
                ]
            },
            ...
        ],
        "objects": [
            {
                "name": <object name>,
                "type": <object type name>
            },
            ...
        ],
        "types_to_supertypes": {
            <type_name>: <supertype_name>,
            ...
        }
    }
    """
    return _get_problem_info_from_tarski(semantics)


def _get_problem_info_from_tarski(semantics):
    reader = PDDLReader(raise_on_error=True)
    reader.parse_domain(config.DOMAIN_DOCUMENT_FILE)
    problem = reader.parse_instance(config.PROBLEM_DOCUMENT_FILE)
    if isinstance(problem.goal, Atom):
        goal_atoms = [problem.goal]
    else:
        goal_atoms = problem.goal.subformulas
    goal_strs = []
    for goal_atom in goal_atoms:
        current_goal_str = f"({goal_atom.predicate.name}"
        for subterm in goal_atom.subterms:
            current_goal_str += f" {subterm.name}"

        current_goal_str += ")"
        goal_strs.append(current_goal_str)

    types_to_supertypes = {}
    for _type, supertype in problem.language.immediate_parent.items():
        types_to_supertypes[_type.name] = None if supertype is None else supertype.name

    print("goal_strs before nlg:", goal_strs)
    # TODO assuming all goals are positive because negative goals are not parsed correctly
    natural_goal_str = nlg.get_natural_language_multiple_predicates(semantics, goal_strs, [False for _ in goal_strs])
    print("natural_goal_str after nlg:", natural_goal_str)

    actions = []
    for tarski_action in problem.actions.values():
        action = {"name": tarski_action.name, "params": []}
        for tarski_param in tarski_action.parameters.variables.values():
            param = {"name": tarski_param.symbol, "type": tarski_param.sort.name}
            action["params"].append(param)
        actions.append(action)

    objects = []
    for tarski_object in problem.language.constants():
        obj = {"name": tarski_object.name, "type": tarski_object.sort.name}
        objects.append(obj)

    return {
        "goal_str": natural_goal_str,
        "actions": actions,
        "objects": objects,
        "types_to_supertypes": types_to_supertypes
    }
