import copy
from src.tmp_exceptions import *
import time
import Config
from collections import OrderedDict

class LLActionSpecV2(object):

    def __init__(self, name, precondition, effect, updates_ll_state = False,execution_sequence=None,hl_args = None,ll_args = None):
        self.name = name
        self.precondition = precondition
        self.effect = effect
        self.updates_ll_state = updates_ll_state
        self.i = 0
        self.j = 0
        self.predicates = None
        self.high_level_arguments = None
        self.low_level_state = None
        self.backtrack_on_failure = None
        self.generated_values = None
        self.generator_object = None
        self.exec_sequence = execution_sequence
        self.hl_args = hl_args
        self.ll_args = ll_args
        self.last_generated_predicate = 0
        self.last_stable_state =None
        self.init_values = None
        self.new_generated_values = None

    def __deepcopy__(self, memodict={}):
        temp = LLActionSpecV2(copy.deepcopy(self.name), copy.deepcopy(self.precondition), copy.deepcopy(self.effect),execution_sequence=copy.deepcopy(self.exec_sequence),hl_args=copy.deepcopy(self.hl_args))
        if "partial_copy" not in memodict.keys():
            temp.generated_values = copy.deepcopy(self.generated_values)
        temp.init_values = copy.deepcopy(self.init_values)
        temp.last_generated_predicate = copy.deepcopy(self.last_generated_predicate)
        temp.last_stable_state = copy.deepcopy(self.last_stable_state)
        return temp

    def reset_argument_generators(self):
        for predicate in self.precondition.get_predicates():
            for argument in predicate.get_arguments():
                argument.reset_generator()
        self.i = 0
        self.j = 0
        self.last_generated_predicate = None
        self.last_stable_state = None


    def generator(self,low_level_state, backtrack_on_failure=True,start_time = None):
        self.start_time = time.time()
        if backtrack_on_failure == False:
            pass
        while True:
            if self.generated_values is None:
                generated_values = {}
            else:
                generated_values = self.generated_values
            if self.new_generated_values is None:
                new_generated_values = OrderedDict()
            else:
                new_generated_values = copy.deepcopy(self.new_generated_values)
            # TODO To Uncomment this: high_level_arguments must be a map instead of list
            self.predicates = self.precondition.get_predicates()
            if self.last_stable_state is None:
                i = self.i
            else:
                i = self.last_generated_predicate
                self.last_stable_state.sync_simulator()
                low_level_state = copy.deepcopy(self.last_stable_state)
            #print predicates
            while i < len(self.predicates):
                if time.time() - start_time > Config.MAX_TIME:
                    self.i = i
                    self.j = 0
                    self.generated_values = generated_values
                    print "**************TIME OUT**************"
                    raise TimeOutException
                predicate = self.predicates[i]
                print "\n****************PREDICATE********************\n{}".format(predicate)
                print "Action: {} , HL Args : {}".format(self.name,[generated_values[val] for val in self.hl_args])
                j = self.j
                arguments = predicate.get_arguments()
                try:
                    low_level_state_modification_count = 0
                    while j < len(arguments):
                        if time.time() - start_time > Config.MAX_TIME:
                            self.i = i
                            self.j = j
                            self.generated_values = generated_values
                            print "*********TIME OUT*************"
                            raise TimeOutException
                        argument = arguments[j]
                        generated_values_copy = {}

                        for k in generated_values:
                            try:
                                generated_values_copy[k] = copy.deepcopy(generated_values[k])
                            except Exception as e:
                                pass
                                # print "Could not deep-copy: "+str(k)+" into generated_values_copy"

                        for preceding_arg in arguments[:j+1]:
                            if preceding_arg.name_alias is not None:
                                generated_values_copy[preceding_arg.name] = generated_values.get(preceding_arg.name_alias)

                        arg_generator = argument.get_generator(known_argument_values=generated_values_copy, low_level_state=low_level_state)
                        try:
                            if arg_generator is not None:
                                print "Generating Arg: " + argument.name + " Predicate: " + predicate.name + " Action: " + self.name
                                parameters = {"flag" : backtrack_on_failure , "type" : argument.type}
                                argument.value = arg_generator.get_next(parameters)
                                generated_values[argument.name] = argument.value
                                new_generated_values[argument.name] = {"type" : argument.type , "value": argument.value}
                                self.last_generated_predicate = i

                            elif argument.name_alias is not None:
                                print "Using Arg: " + argument.name +" alias: "+str(argument.name_alias)+" Val: "+str(generated_values.get(argument.name_alias))[:100]+ " Predicate: " + predicate.name + " Action: " + self.name
                                assert argument.name in generated_values or argument.name_alias in generated_values, str(argument) + " has no generator Object or name_alias and is not found in prev. generated_values!"
                                # if argument.name_alias is None:
                                #     argument.value = copy.deepcopy(generated_values.get(argument.name))
                                #above changed to single line below, notice name_alias
                                argument.value = copy.deepcopy(generated_values.get(argument.name_alias))
                                new_generated_values[argument.name] = argument.value
                                generated_values[argument.name] = argument.value
                            else:
                                print "Skipping Arg: {}".format(argument.name)

                            if argument.name in self.exec_sequence:
                                self.last_stable_state = copy.deepcopy(low_level_state)
                                low_level_state.apply(argument, generated_values)
                                low_level_state_modification_count = low_level_state_modification_count + 1
                            j = j + 1

                        except StopIteration:
                            argument.reset_generator()
                            if j != 0:
                                j = j - 1
                                print str(j)+" rolling back"
                                if arguments[j] in self.exec_sequence:
                                    low_level_state.rollback()
                                    low_level_state_modification_count -= 1
                                    j -= 1
                                while not arguments[j].has_alternate_values():
                                    if arguments[j].name in self.exec_sequence:
                                        low_level_state.rollback()
                                        low_level_state_modification_count = low_level_state_modification_count - 1
                                    j = j - 1
                                    if j < 0:
                                        raise StopIteration
                            else:
                                raise StopIteration
                    # print predicate.name
                    evaluation, fail_strings = predicate(low_level_state=low_level_state, arg_map = generated_values, flag = backtrack_on_failure)
                    if not evaluation:
                        if not backtrack_on_failure:
                            raise FailedPredicateException(predicate, failure_reason=fail_strings)
                        else:
                            i = i - 1
                            for a in range(self.predicates[i].get_argument_count()):
                                if self.predicates[i].get_arguments()[a].name in self.exec_sequence:
                                    low_level_state.rollback()

                            while not self.predicates[i].has_generatable_arguments():
                                for a in range(self.predicates[i].get_argument_count()):
                                    if self.predicates[i].get_arguments()[a].name in self.exec_sequence:
                                        low_level_state.rollbacks()

                                i = i - 1


                            i = i - 1

                    low_level_state.sync_simulator()

                    i = i + 1
                    self.i =i


                except StopIteration:
                    if i != 0:
                        # for arg in predicate.get_arguments():
                        #     arg.reset_generator()
                        i = i -1
                        for k in range(low_level_state_modification_count):
                            low_level_state.rollback()

                        while not self.predicates[i].has_generatable_arguments():

                            for a in range(self.predicates[i].get_argument_count()):
                                if self.predicates[i].get_arguments()[j].name in self.exec_sequence:
                                    low_level_state.rollback()

                            i = i - 1

                            if i < 0:
                                raise OutOfPossibleErrorsException(None)
                            else:
                                for a in range(self.predicates[i].get_argument_count()):
                                    if self.predicates[i].get_arguments()[a].name in self.exec_sequence:
                                        low_level_state.rollback()
                                    pass

                        print "Rolled back to :"+str(self.predicates[i])
                    else:
                        raise OutOfPossibleErrorsException(predicate)
            self.generated_values = copy.deepcopy(generated_values)
            self.new_generated_values = copy.deepcopy(new_generated_values)
            # self.i = self.i - 1
            self.j = 0
            for effect in self.effect.getPositivePredicates():
                # effect.apply(generated_values,low_level_state)
                low_level_state.apply_effect(effect, generated_values)
            yield new_generated_values,generated_values, low_level_state
