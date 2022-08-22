import logging
import os
import yaml
import pddlpy
import copy
import config

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
from Search import *
from natural_language import natural_language_generator as nlg

TEMP_DOM_FILE = FILE_DIR + '/temp/dom.pddl'
TEMP_PROB_FILE = FILE_DIR + '/temp/prob.pddl'
TEMP_PLAN_FILE = FILE_DIR + '/temp/plan.sol'
VAL_COMMAND = FILE_DIR + '/scripts/valplan.sh {} {} {} ' + config.VAL_PATH
VAL_INFO_COMMAND = FILE_DIR + '/scripts/val_info.sh {} {} {} ' + config.VAL_PATH
ACTION_DEF_STR = '(:action {}\n:parameters ({})\n:precondition\n(and\n{}\n)\n:effect\n(and\n{}\n)\n)\n'
log = logging.getLogger(__name__)


class Problem:

    def __init__(self, domain_file, problem_file, foil_prefix, foil_count, lattice_file,
                 dom_templ, prob_templ, semantics):
        self.lattice_file = lattice_file
        dom_prob = pddlpy.DomainProblem(domain_file, problem_file)
        self.resolved_models = {}
        self.orig_start = self.convert_prop_tuple_list(dom_prob.initialstate())
        self.goal_state = self.convert_prop_tuple_list(dom_prob.goals())
        self.orig_dom_map = self.convert_domain_obj_map(dom_prob)
        self.concret_costs = {}
        self.generate_lattice_from_file()
        self.foil = []
        for f in range(1, int(foil_count) + 1):
            foil_file = foil_prefix + str(f)
            with open(foil_file) as f_fd:
                self.foil.append([i.strip() for i in f_fd.readlines()])
        with open(dom_templ) as d_fd:
            self.domain_template_str = d_fd.read().strip()
        with open(prob_templ) as p_fd:
            self.prob_template_str = p_fd.read().strip()
        self.semantics = semantics

    def write_plan_sol(self, foil, plan_file):
        with open(plan_file, 'w') as p_fd:
            plan_str_list = []
            for act_ind in range(len(foil)):
                plan_str_list.append(str(act_ind + 1) + ": " + foil[act_ind])

            p_fd.write("\n".join(plan_str_list))

    def convert_prop_tuple_list(self, orig_prop_list, para_type=None):
        prop_list = set()
        for p in orig_prop_list:
            if type(p) is tuple:
                prop = ' '.join([str(i).lower() for i in p])
            else:
                prop = ' '.join([str(i).lower() for i in p.predicate])
            prop_list.add(prop)
        return prop_list

    def convert_domain_obj_map(self, prob_object):
        dom_map = {}
        for act in (prob_object.operators()):
            sorted_var_name = list(prob_object.domain.operators[act].variable_list.keys())
            sorted_var_name.sort()
            para_list = []
            for k in sorted_var_name:
                if prob_object.domain.operators[act].variable_list[k]:
                    para_list.append(k + " - " + prob_object.domain.operators[act].variable_list[k])
                else:
                    para_list.append(k)
            action_name = act
            dom_map[action_name] = {}
            dom_map[action_name]['parameters'] = " ".join(para_list)
            dom_map[action_name]['precondition_pos'] = self.convert_prop_tuple_list(
                prob_object.domain.operators[act].precondition_pos)
            dom_map[action_name]['precondition_neg'] = self.convert_prop_tuple_list(
                prob_object.domain.operators[act].precondition_neg)
            dom_map[action_name]['effect_pos'] = self.convert_prop_tuple_list(
                prob_object.domain.operators[act].effect_pos)
            dom_map[action_name]['effect_neg'] = self.convert_prop_tuple_list(
                prob_object.domain.operators[act].effect_neg)

        dom_map['problem'] = {}
        dom_map['problem']['init'] = copy.deepcopy(self.orig_start)
        dom_map['problem']['goal'] = copy.deepcopy(self.goal_state)
        dom_map['current_cost'] = 0
        return dom_map

    def ground_all_operators(self, prob_object):
        ground_operators = []
        for act in prob_object.operators():
            ground_operators += list(prob_object.ground_operator(act))
        return ground_operators

    def generate_lattice_from_file(self):
        with open(self.lattice_file, 'r') as l_fd:
            lattice_map = yaml.load(l_fd)
        self.node_map = {}
        self.node_map[lattice_map['Lattice']['Init']] = copy.deepcopy(self.orig_dom_map)
        self.init_node = lattice_map['Lattice']['Init']
        self.edges = lattice_map['Lattice']['Edge_map']
        self.inverse_edges = lattice_map['Lattice']['Inv_edge_map']
        self.node_pred_map = lattice_map['Lattice']['Node_map']
        self.sup_node = lattice_map['Lattice']['Sup']
        for nd in lattice_map['Lattice']['Nodes']:
            if nd not in self.node_map:
                self.node_map[nd] = None

    def set_model_for_node(self, node):
        if self.node_map[node]:
            return True
        self.node_map[node] = copy.deepcopy(self.orig_dom_map)
        for pred in self.node_pred_map[node]:
            self.assign_cost_to_pred_at_node(pred, node)
        return True

    def assign_cost_to_pred_at_node(self, pred, node):
        cost_delta = 0
        for ky in self.node_map[node]:
            if ky != "current_cost":
                for k2 in self.node_map[node][ky]:
                    for p in self.orig_dom_map[ky][k2]:
                        if pred.lower() == p.split(' ')[0].lower() and p in self.node_map[node][ky][k2]:
                            self.node_map[node][ky][k2].remove(p)
                            cost_delta += 1
        if pred not in self.concret_costs.keys():
            self.concret_costs[pred] = cost_delta

    def find_most_abstract_models(self):
        model = self.sup_node
        if self.test_foil_condition_neg(model):
            return [model]
        else:
            return []

    def find_operator_name(self, action):
        return action.replace('(', '').replace(')', '').strip().split(' ')[0]

    def find_action_args(self, action):
        return action.replace('(', '').replace(')', '').strip().split(' ')[1:]

    def find_var_to_obj_map(self, var_list_raw, obj_list):

        v = var_list_raw.split('?')
        var_list = []
        for x in v:
            if "-" in x:
                var_list.append(x.split("-")[1])

        assert len(var_list) == len(obj_list)
        return {var_list[i]: obj_list[i] for i in range(len(var_list))}

    def ground_operator_defn(self, defn, action_name):
        var_to_obj_map = self.find_var_to_obj_map(defn['parameters'], self.find_action_args(action_name))
        new_def = {}
        for part in defn:
            if part != 'parameters':
                new_def[part] = set()
                for fl in defn[part]:
                    new_fl = fl
                    for var in var_to_obj_map:
                        new_fl = new_fl.replace(var, var_to_obj_map[var])
                    new_def[part].add(new_fl)
            else:
                new_def['parameters'] = []
        return new_def

    def find_start_state(self):
        # also include checks for other restrictions
        # TODO what does that comment mean ^
        self.start_state_belief_space = set(self.find_most_abstract_models())

    def make_problem_domain_file(self, curr_model, dom_file, prob_file):
        self.set_model_for_node(curr_model)
        action_strings = []
        for act in self.node_map[curr_model].keys():
            if act != "problem" and act != "current_cost":
                precondition_list = ['(' + i + ')' for i in self.node_map[curr_model][act]['precondition_pos']]
                precondition_list += ['(not (' + i + '))' for i in self.node_map[curr_model][act]['precondition_neg']]
                effect_list = ['(' + i + ')' for i in self.node_map[curr_model][act]['effect_pos']]
                effect_list += ['(not (' + i + '))' for i in self.node_map[curr_model][act]['effect_neg']]
                action_strings.append(ACTION_DEF_STR.format(act, self.node_map[curr_model][act]['parameters'],
                                                            "\n".join(precondition_list), "\n".join(effect_list)))
        dom_str = self.domain_template_str.format("\n".join(action_strings))

        goal_state_str_list = ['(' + i + ')' for i in self.node_map[curr_model]['problem']['goal']]
        init_state_str_list = ['(' + i + ')' for i in self.node_map[curr_model]['problem']['init']]
        prob_str = self.prob_template_str.format("\n".join(init_state_str_list), "\n".join(goal_state_str_list))

        with open(dom_file, 'w') as d_fd:
            d_fd.write(dom_str)

        with open(prob_file, 'w') as p_fd:
            p_fd.write(prob_str)

    # TODO give this and the other one a more descriptive name
    def test_foil_condition_neg(self, curr_model):
        # return true if goal is empty here
        self.make_problem_domain_file(curr_model, TEMP_DOM_FILE, TEMP_PROB_FILE)
        for i in range(len(self.foil)):
            self.write_plan_sol(self.foil[i], TEMP_PLAN_FILE)
            output = [i.strip() for i in
                      os.popen(VAL_COMMAND.format(TEMP_DOM_FILE, TEMP_PROB_FILE, TEMP_PLAN_FILE)).read().strip().split('\n')]
            if not eval(output[0]):
                return False
        return True

    def test_foil_condition_pos(self, curr_model):
        # return true if goal is empty here
        self.make_problem_domain_file(curr_model, TEMP_DOM_FILE, TEMP_PROB_FILE)
        for i in range(len(self.foil)):
            self.write_plan_sol(self.foil[i], TEMP_PLAN_FILE)
            output = [i.strip() for i in
                      os.popen(VAL_COMMAND.format(TEMP_DOM_FILE, TEMP_PROB_FILE, TEMP_PLAN_FILE)).read().strip().split('\n')]
            if eval(output[0]):
                return False
        return True

    def find_unresolved_foils(self, curr_model, current_foils):
        unreslv_foils = set()
        self.make_problem_domain_file(curr_model, TEMP_DOM_FILE, TEMP_PROB_FILE)
        for f in current_foils:
            self.write_plan_sol(f.split('@'), TEMP_PLAN_FILE)
            output = [i.strip() for i in
                      os.popen(VAL_COMMAND.format(TEMP_DOM_FILE, TEMP_PROB_FILE, TEMP_PLAN_FILE)).read().strip().split('\n')]

            if eval(output[0]):
                unreslv_foils.add(copy.deepcopy(f))
        return unreslv_foils

    def run_blind_conformant(self):
        # Run a pretest to see if the foil is executable in the most concrete model
        unres_fls = self.find_unresolved_foils(self.init_node, set('@'.join(i) for i in self.foil))
        if len(unres_fls) == len(self.foil):
            log.debug("All foils are valid as given")
            return {"failed": False}

        # Make a node for start state
        start_node = AbsNode(self, self.start_state_belief_space, [], set('@'.join(i) for i in self.foil))

        # Start search
        exp_node = greedy_search(start_node)

        # TODO: Assuming there is a single node
        explanation_map = {}
        explanation_map["pred_to_be_explained"] = exp_node.get_plan()[0]
        # TODO: Find model information
        model_info = {}
        curr_model = list(exp_node.get_state())[0]
        for act in self.node_map[curr_model].keys():
            if act != "problem" and act != "current_cost":
                tmp_list = []
                for key in self.node_map[curr_model][act]:
                    for pred in self.node_map[curr_model][act][key]:
                        if explanation_map["pred_to_be_explained"] in pred:
                            tmp_list += [key]
                if len(tmp_list) != 0:
                    model_info[act] = tmp_list

        explanation_map["model_information"] = model_info
        # TODO: Assuming a single foil
        # Key, Value == Action no, Explanation
        error_trace = dict()
        self.make_problem_domain_file(curr_model, TEMP_DOM_FILE, TEMP_PROB_FILE)
        for f in self.foil:
            log.debug(f"Putting together explanation for foil: {f}")
            self.write_plan_sol(f, TEMP_PLAN_FILE)

            output = [i.strip() for i in
                      os.popen(VAL_INFO_COMMAND.format(TEMP_DOM_FILE, TEMP_PROB_FILE, TEMP_PLAN_FILE)).read().strip().split('\n')]
            fail_info = [o.strip() for o in output[0].split("@")]
            log.debug("fail_info:{}".format(fail_info))
            failure_type = fail_info[0]
            if failure_type == "precondition" or failure_type == "negated-precondition":
                failed_step_str = fail_info[1]
                failed_step = int(failed_step_str)
                failed_action = fail_info[-2]
                failed_action_display = nlg.get_natural_language_action(self.semantics, failed_action)
                failed_precondition = fail_info[-1]
                # reversed due to the fact that the precondition was not met
                negated = False if failure_type == "negated-precondition" else True
                failed_precondition_display = nlg.get_natural_language_single_predicate(
                    self.semantics,
                    failed_precondition,
                    negated
                )
                failed_prop = set([failed_precondition.replace('(', '').replace(')', '')])
                failure_cause = 'The action at step ' + failed_step_str + ' (' + failed_action_display +\
                                ') could not be performed because ' + failed_precondition_display + "."
                explanation_map["failure_cause"] = failure_cause
                error_trace[failed_step - 1] = failure_cause
                explanation_map["failed_precondition"] = failed_precondition
            elif failure_type == "goal":
                failed_step = len(self.foil[0])
                failed_goals = [f.strip() for f in fail_info[-1].split('#') if f != ""]
                failed_goals = [(f[3:].strip() if f.startswith("and") else f) for f in failed_goals]
                failed_goals_display = nlg.get_natural_language_multiple_predicates(
                    self.semantics,
                    failed_goals,
                    # TODO assuming all goals are positive because negative goals are not parsed correctly
                    #  (a positive goal must be negated here because it wasn't reached)
                    [True for _ in failed_goals]
                )
                failed_prop = set([p.replace('(', '').replace(')', '') for p in failed_goals])
                failure_cause = "The goal state is not achieved! " + failed_goals_display

                explanation_map["failure_cause"] = failure_cause
                error_trace[failed_step - 1] = failure_cause
                explanation_map["failed_precondition"] = "N/A"
            else:
                raise Exception(f"The VAL script did not give the expected output! Specifically, the first bit of output was '{failure_type}', but was expected to be the failure type of 'precondition', 'negated-precondition', or 'goal'.")

        """if failed_prop in self.node_list[curr_model]["problem"]["init"]:
            error_trace[-1] = "The proposition {} is true in the initial state".format(failed_prop)
        else:
            error_trace[-1] = "The proposition {} is false in the initial state".format(failed_prop)"""
        # Last action that negated the pre-condition for the failed action
        last_negating_action = None
        for plan in self.foil:
            for ac_ind in range(len(plan)):
                ac = plan[ac_ind]
                if ac_ind < (failed_step - 1):
                    curr_operator = self.find_operator_name(ac)
                    lifted_defn = copy.deepcopy(self.node_map[curr_model][curr_operator])
                    grounded_defn = self.ground_operator_defn(lifted_defn, ac)
                    for add_fl in grounded_defn["effect_pos"]:
                        if add_fl in failed_prop:
                            error_trace[ac_ind] = "Action {} sets the fluent {} true".format(ac, add_fl)
                    for add_fl in grounded_defn["effect_neg"]:
                        if add_fl in failed_prop:
                            last_negating_action = ac
                            error_trace[ac_ind] = "Action {} sets the fluent {} false".format(ac, add_fl)

        # Checking if we have an action that negated the pre-condition
        if last_negating_action:
            error_trace[failed_step - 1] += " got negated by the last action {}.".format(last_negating_action)
        else:
            error_trace[failed_step - 1] += " was false in the initial state."
        explanation_map["error_trace"] = error_trace
        explanation_map["failed"] = True
        log.debug(f"explanation_map:{explanation_map}")
        return explanation_map

    def explain(self):
        self.find_start_state()
        # there will be options here
        return self.run_blind_conformant()
