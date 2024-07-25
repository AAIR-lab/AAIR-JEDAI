import copy
from src.Functions.Function import Function
class Predicate(Function):
    def __init__(self, name, arg_list=None):
        self.name = name
        self.arg_list = arg_list
        self.proposition_string = []


    def __deepcopy__(self, memodict={}):
        return Predicate(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def __repr__(self):
        return self.name + "(" + str(self.arg_list)+")"

    def __call__(self, *args, **kwargs):
        return True, []

    def get_arguments(self):
        return self.arg_list

    def has_generatable_arguments(self):
        for argument in self.arg_list:
            if argument.has_alternate_values():
                return True
        return False

    def get_argument_count(self):
        return len(self.arg_list)

    def get_arg_by_name(self, name):
        for arg in self.arg_list:
            if arg.name == name:
                return arg
        return None

    def apply(self,ll_state,generated_values):
        raise NotImplementedError

    def evaluate(self, *argv):
        return True

    def set_proposition_string(self,proposition_strings):
        self.proposition_string = proposition_strings


    def get_proposition_strings(self):
        return self.proposition_string