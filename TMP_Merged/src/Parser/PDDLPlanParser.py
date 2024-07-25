from src.Parser.InitFileMgr import *
import re
import itertools
import StringIO

print_progress = False # print grounded props for each action and state after each action

class PDDLActionParserException(Exception):
    def __init__(self, msg):
        self.msg = msg
        
class PDDLPlanParser(object):
    def __init__(self, pddlDomainFileName, pddlProblemFileName, planFileName):
        with open(pddlDomainFileName, 'r') as f:
            self.domain = f.read().lower()
        if isinstance(planFileName, StringIO.StringIO):
            planFileName.seek(0)
            self.plan = planFileName.read().lower()
        else:
            with open(planFileName, 'r') as f:
                self.plan = f.read().lower()
        self.state = InitFileMgr(pddlProblemFileName).getCurrentState()
        self.typing_used = None not in self.state.getObjDict().keys()
        self.plan_list = []
        self.state_list = []

    def get_pddl_states(self):
        # Top-level function.

        try:
            self.handle_constants()
            if self.typing_used:
                # self.handle_subtyping()
                pass
            self.construct_plan_list()
            self.parse_effects()
            return self.state_list
        except PDDLActionParserException as e:
            print("Error: " + e.msg + " Quitting.")

    def parse_effects(self):
        # Second-top function. For each action in plan_list, constructs a symbol table, constructs
        # effect literals, and adds grounded action propositions.

        if print_progress:
            print("Initial state:")
            self.state.printState()
            raw_input()
        self.state_list.append(self.state.getStateCopy())
        for action_name, action_args in self.plan_list:
            actionstr = self.get_actionstr(action_name)
            symbol_table = self.construct_symbol_table(action_name, actionstr, action_args)
            grounded_props = self.process_effect(action_name, actionstr, symbol_table)
            if print_progress:
                print("Grounded props about to be added for " + action_name + ":\nLength: " + repr(len(grounded_props)))
                print(grounded_props)
                raw_input()
            self.state.addProps(grounded_props)
            self.state_list.append(self.state.getStateCopy())
            if print_progress:
                print("State after action " + action_name + ":")
                self.state.printState()
                raw_input()

    def handle_constants(self):
        # Parses domain file. Modifies object dict to handle constants.
        const_str = self.domain
        objDict = self.state.getObjDict()
        constant_match = re.search(":constants.+?\)", const_str, flags=re.DOTALL)
        if constant_match is not None:
            const_str = constant_match.group()[10:-1].strip()
        else:
            const_str = ""
        if self.typing_used:
            const_match = re.search("-", const_str)
            while const_match is not None:
                const_names = const_str[:const_match.start()]
                const_str = const_str[const_match.end():].lstrip()
                rest = re.search("\s", const_str)
                const_type = const_str[:rest.start()] if rest else const_str
                
                const_names = re.split("\s", const_names) # strip out space characters
                const_names = list(filter(lambda x: x != "" and not x.startswith(";"), const_names))
                
                if const_type in objDict.keys():
                    objDict[const_type].extend(const_names)
                else:
                    objDict[const_type] = const_names
                    
                const_str = const_str[rest.end():] if rest else ""
                const_match = re.search("-", const_str)
        else:
            const_names = re.split("\s", const_str) # strip out space characters
            const_names = list(filter(lambda x: x != "" and not x.startswith(";"), const_names))
            if None in objDict.keys():
                objDict[None].extend(const_names)
            else:
                objDict[None] = const_names

    def handle_subtyping(self):
        # Parses domain file. Modifies object dict to handle subtyping.

        not_done = True

        while not_done:
            not_done = False
            objdict = self.state.getObjDict()
            typestr = self.domain
            typestr_match = re.search(":types.+?\)", typestr, flags=re.DOTALL)
            if not typestr_match:
                raise PDDLActionParserException("No types defined in domain file!")
            typestr = typestr_match.group()[6:-1]

            type_match = re.search("-", typestr)
            while type_match is not None:
                subtypes = typestr[:type_match.start()]
                typestr = typestr[type_match.end():].lstrip()
                rest = re.search("\s", typestr)
                subtype_class = typestr[:rest.start()] if rest else typestr

                subtypes = re.split("\s", subtypes) # strip out space characters
                subtypes = list(filter(lambda x: x != "" and not x.startswith(";"), subtypes))

                if subtype_class not in objdict.keys():
                    not_done = True
                    unknown_subtypes = list(filter(lambda subtype: subtype not in objdict.keys(), subtypes))
                    if len(unknown_subtypes) == 0:
                        objdict[subtype_class] = list(itertools.chain.from_iterable([objdict[subtype] for subtype in subtypes]))

                typestr = typestr[rest.end():] if rest else ""
                type_match = re.search("-", typestr)
            
    def construct_plan_list(self):
        # Parses plan file. Constructs plan list:
        # [[action_name, [args]], ...]

        plan_lines = self.plan.split("\n")
        for line in plan_lines:
            colon_match = re.search(":", line)
            if colon_match:
                line = line[colon_match.end():].lstrip() # keep only everything after the colon
                words = line.lower().split()
                if len(words) > 0:
                    self.plan_list.append([words[0], words[1:]])

    def _get_balanced_string(self, str, init_paren_count, error_msg="something"):
        # Returns first fully balanced expression in str, assuming the first
        # init_paren_count open parentheses have already been cut off. If called
        # with init_paren_count > 0, will retain corresponding close parentheses,
        # so those must be spliced off. Also returns rest of string, left stripped.

        paren_count = init_paren_count
        started, index = False, 0
        for char in str:
            if char == "(":
                started = True
                paren_count += 1
            if char == ")":
                started = True
                paren_count -= 1
            index += 1
            if started and paren_count == 0:
                return str[:index], str[index:].lstrip()
        raise PDDLActionParserException("Reached end of file parsing for " + error_msg + "!")

    def get_actionstr(self, action_name):
        # Parses domain file and retrieves string form of action action_name.

        actionstr = self.domain
        actionstr_start = re.search(":action\s*" + action_name + "\s", actionstr, flags=re.IGNORECASE)
        if not actionstr_start:
            raise PDDLActionParserException("No action with name " + repr(action_name) + " found!")
        actionstr = actionstr[actionstr_start.end():]
        return self._get_balanced_string(actionstr, 1, "action " + repr(action_name))[0]

    def construct_symbol_table(self, action_name, actionstr, action_args):
        # Parses actionstr for params list. Returns mapping between params and args:
        # {param: arg}

        params_start = re.search(":parameters\s*\(", actionstr, flags=re.IGNORECASE)
        if not params_start:
            raise PDDLActionParserException("No parameters in action " + repr(action_name) + "!")
        params = actionstr[params_start.end():]
        params_end = re.search("\)", params)
        params = params[:params_end.start()]

        paramslist = re.split("\s+", params)
        paramslist = list(filter(lambda x: x.startswith("?"), paramslist)) # keep only the param names, which start with ?

        if not len(paramslist) == len(action_args):
            raise PDDLActionParserException("Number of params and args do not match for action " + repr(action_name) + "!")
        return {paramslist[i]:action_args[i] for i in range(len(paramslist))}

    def process_effect(self, action_name, actionstr, symbol_table):
        # Parses actionstr for effect list, then returns grounded propositions.
        
        parametrized_props = []
        effectstr_start = re.search(":effect", actionstr, flags=re.IGNORECASE)
        if not effectstr_start:
            raise PDDLActionParserException("No effect for action " + repr(action_name) + "!")
        effectstr = actionstr[effectstr_start.end():]
        effectstr = self._get_balanced_string(effectstr, 0, "effect for action " + repr(action_name))[0].strip()

        return self.generate_grounded_effects(effectstr, symbol_table)

    def generate_grounded_effects(self, str, symbol_table):
        # Calls process_and_or, process_not, process_when, or process_forall according to what's next in str.
        # If none of these, must be just a single proposition, so ground it with symbol_table.
        # Returns list of grounded propositions to be added to state.

        if re.match("\s*;", str):
            return []
        grounded_props = []
        and_match = re.match("\(\s*and\s*\(", str, flags=re.IGNORECASE)
        or_match = re.match("\(\s*or\s*\(", str, flags=re.IGNORECASE)
        not_match = re.match("\(\s*not\s*\(", str, flags=re.IGNORECASE)
        when_match = re.match("\(\s*when\s*\(", str, flags=re.IGNORECASE)
        forall_match = re.match("\(\s*forall\s*\(", str, flags=re.IGNORECASE)
        increase_match = re.match("\(\s*increase\s*\(", str, flags=re.IGNORECASE)
        get_propstr = lambda str, match: self._get_balanced_string(str[(match.end()-1):], 1)[0].strip()[:-1].strip() # strips off beginning keyword

        if and_match:
            grounded_props.extend(self.process_and(get_propstr(str, and_match), symbol_table, generate=True))
        elif or_match:
            grounded_props.extend(self.process_or(get_propstr(str, or_match), symbol_table, generate=True))
        elif not_match:
            grounded_props.extend(self.process_not(get_propstr(str, not_match), symbol_table, generate=True))
        elif when_match:
            grounded_props.extend(self.process_when(get_propstr(str, when_match), symbol_table))
        elif forall_match:
            grounded_props.extend(self.process_forall(get_propstr(str, forall_match), symbol_table, generate=True))
        elif increase_match:
            pass # ignore "increase" statements
        else:       
            grounded_props.append(self.get_grounded_proposition(str, symbol_table))

        return grounded_props

    def evaluate_grounded_effects(self, str, symbol_table):
        # Calls process_and_or, process_not, process_when, or process_forall according to what's next in str.
        # Evaluates truth or falsity of grounded propositions. Used for when condition evaluation.

        if re.match("\s*;", str):
            raise PDDLActionParserException("Comment in condition evaluation.")
        and_match = re.match("\(\s*and\s*\(", str, flags=re.IGNORECASE)
        or_match = re.match("\(\s*or\s*\(", str, flags=re.IGNORECASE)
        not_match = re.match("\(\s*not\s*\(", str, flags=re.IGNORECASE)
        when_match = re.match("\(\s*when\s*\(", str, flags=re.IGNORECASE)
        forall_match = re.match("\(\s*forall\s*\(", str, flags=re.IGNORECASE)
        increase_match = re.match("\(\s*increase\s*\(", str, flags=re.IGNORECASE)
        get_propstr = lambda str, match: self._get_balanced_string(str[(match.end()-1):], 1)[0].strip()[:-1].strip() # strips off beginning keyword
        
        if and_match:
            return self.process_and(get_propstr(str, and_match), symbol_table, generate=False)
        elif or_match:
            return self.process_or(get_propstr(str, or_match), symbol_table, generate=False)
        elif not_match:
            return self.process_not(get_propstr(str, not_match), symbol_table, generate=False)
        elif when_match:
            raise PDDLActionParserException("Nested when statements!")
        elif forall_match:
            return self.process_forall(get_propstr(str, forall_match), symbol_table, generate=False)
        elif increase_match:
            pass # ignore "increase" statements
        else:
            return self.get_grounded_proposition(str, symbol_table) in self.state.getTrueProps()

    def process_and(self, propstr, symbol_table, generate):
        grounded_props = []
        while propstr != "":
            param_prop, propstr = self._get_balanced_string(propstr, 0)
            if generate:
                grounded_props.extend(self.generate_grounded_effects(param_prop, symbol_table))
            else:
                if not self.evaluate_grounded_effects(param_prop, symbol_table):
                    return False

        return grounded_props if generate else True

    def process_or(self, propstr, symbol_table, generate):
        grounded_props = []
        
        while propstr != "":
            param_prop, propstr = self._get_balanced_string(propstr, 0)
            if generate:
                grounded_props.extend(self.generate_grounded_effects(param_prop, symbol_table))
            else:
                if self.evaluate_grounded_effects(param_prop, symbol_table):
                    return True

        return grounded_props if generate else False

    def process_not(self, propstr, symbol_table, generate):
        if generate: # we can add this literal into state, with not in front
            grounded_props = self.generate_grounded_effects(propstr, symbol_table)
            if len(grounded_props) > 1:
                raise PDDLActionParserException("'Not' followed by something other than atom outside condition evaluation!")
            return ["(not " + grounded_props[0] + ")"]
        else:
            return not self.evaluate_grounded_effects(propstr, symbol_table)

    def _process_when_condition(self, condition_str, symbol_table):
        return self.evaluate_grounded_effects(condition_str, symbol_table)

    def process_when(self, propstr, symbol_table):
        condition_str, body = self._get_balanced_string(propstr, 0)

        if self._process_when_condition(condition_str, symbol_table):
            return self.generate_grounded_effects(body, symbol_table)
        else:
            return []

    def process_forall(self, propstr, symbol_table, generate):
        grounded_props = []
        augmented_table = symbol_table.copy()
        
        try:
            if self.typing_used:
                quantifier_name, quantifier_type = self._get_balanced_string(propstr, 0)[0].split("-")
                quantifier_name = quantifier_name[1:].strip() # remove (
                quantifier_type = quantifier_type[:-1].strip() # remove )
                quantifier_values = self.state.getObjDict()[quantifier_type]
            else:
                quantifier_name = self._get_balanced_string(propstr, 0)[0]
                quantifier_name = quantifier_name[1:-1].strip() # remove ()
                quantifier_values = self.state.getObjDict()[None]
        except (ValueError, KeyError):
            raise PDDLActionParserException("Syntax error in forall statement!")

        forall_body = self._get_balanced_string(propstr, 0)[1]
        for possibility in quantifier_values:
            augmented_table[quantifier_name] = possibility
            if generate:
                grounded_props.extend(self.generate_grounded_effects(forall_body, augmented_table))
            else:
                if not self.evaluate_grounded_effects(forall_body, augmented_table):
                    return False

        return grounded_props if generate else True

    def get_grounded_proposition(self, propstr, symbol_table):
        # Replace parameters with arguments in parametrized
        # proposition propstr, using symbol_table for lookup.

        # clean up expression whitespace
        propstr = "(" + propstr[1:-1].strip() + ")"
        propstr = re.sub("\s*\?", " ?", propstr)
        propstr = re.sub("\s*\)", ")", propstr)

        try:
            replacer = lambda match: symbol_table[match.group()[:-1]] + match.group()[-1] # remove the terminating character after ?<param_name> for lookup
            return re.sub("\?.+?[\)\ \n\t]", replacer, propstr) # params are assumed to have ) or whitespace afterward when they appear in action effects
        except KeyError:
            raise PDDLActionParserException("Could not find a binding in proposition string " + propstr + "!")

if __name__ == "__main__":
    parser = PDDLPlanParser(pddlDomainFileName="/home/midhun/Documents/TMP_Merged/SampleTasks/cup-washer.pddl",
                            pddlProblemFileName="/home/midhun/Documents/TMP_Merged/SampleTasks/cup-washer.problem",
                            planFileName="/tmp/test_plan")
    a = parser.get_pddl_states()
