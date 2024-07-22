#import pddlpy
from src.Wrappers.Argument import Argument

from src.DataStructures.Predicate import Predicate


class PDDLParser(object):
    def __init__(self, domain_file, problem_file):
        self.domprob = pddlpy.DomainProblem(domain_file, problem_file)

    def get_action_names(self):
        '''
        :return: list of action names as strings
        '''
        return self.domprob.domain.operators.keys()

    def get_initial_state(self):
        '''
        :return: A  set of predicate tuples
        Eg: {(u'At', u'object0', u'loc_object0'),
 (u'RobotAt', u'robotInitLoc')}
        Call the predicate function of each tuple instance to get a list of predicates
        '''
        return self.domprob.initialstate()


    def parse_action(self, action_name):
        '''
        :param action_name:
        :return:
        '''
        args = self.get_arguments_objects(action_name)
        precondition_pos = self.get_precondition_positive(action_name)
        precondition_neg = self.get_precondition_negative(action_name)
        effect_pos = self.get_effect_positive(action_name)
        effect_neg = self.get_effect_negative(action_name)

        return args, precondition_pos, precondition_neg, effect_pos, effect_neg

    def get_arguments_objects(self, action_name):
        '''
        :param action_name:
        :return: List of [ Argument() ]
        '''

        variable_list = self.get_parameters(action_name)
        arg_objects = []
        for arg_name in variable_list:
            arg_type = variable_list.get(arg_name)
            arg_objects.append(Argument(arg_name, arg_type, value=None))

        return arg_objects


    def get_parameters(self, action_name):
        '''
        :param action_name:
        :return: dict { variable_str: type_str}
        '''
        action = self.domprob.domain.operators.get(action_name)
        return action.variable_list


    def get_precondition_positive(self, action_name):
        return self.__get_predicates(action_name, type='precondition', kind='positive')

    def get_precondition_negative(self, action_name):
        return self.__get_predicates(action_name, type='precondition', kind='negative')

    def get_effect_positive(self, action_name):
        return self.__get_predicates(action_name, type='effect', kind='positive')

    def get_effect_negative(self, action_name):
        return self.__get_predicates(action_name, type='effect', kind='negative')

    def __get_predicates(self, action_name, type, kind):
        '''

        :param action_name:
        :param type: 'precondition' or 'effect'
        :param kind: 'positive' or 'negative'
        :return:
        '''
        param_type_dict = self.get_parameters(action_name)

        action = self.domprob.domain.operators.get(action_name)

        assert action is not None, "Could not find action :"+action_name

        if type == 'precondition':
            if kind == 'positive':
                predicate_conjunction = action.precondition_pos
            elif kind == 'negative':
                predicate_conjunction = action.precondition_neg
        elif type == 'effect':
            if kind == 'positive':
                predicate_conjunction = action.effect_pos
            elif kind == 'negative':
                predicate_conjunction = action.effect_neg


        precondition_predicates_list = []
        for predicate_tuple in predicate_conjunction:
            predicate = predicate_tuple.predicate
            predicate_name  = predicate[0]
            arg_obj_list = []

            for arg_str in predicate[1:]:
                arg = Argument(name=arg_str, type=param_type_dict.get(arg_str,None), value=None)
                arg_obj_list.append(arg)

            predicate = Predicate(predicate_name, arg_obj_list)
            precondition_predicates_list.append(predicate)

        return precondition_predicates_list



if __name__ == "__main__":

    pddlParser = PDDLParser('/home/midhun/Documents/TMP_Merged/SampleTasks/robotics_twoarms_objloc.pddl', '/home/midhun/Documents/TMP_Merged/SampleTasks/robotics_fetch_1_cans_problem.pddl')
    # res = pddlParser.get_preconditions_positive('op1')
    print res
