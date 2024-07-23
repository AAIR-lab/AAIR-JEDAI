from src.DataStructures.Predicate import Predicate
import copy

class BatterySufficient(Predicate):
    def __init__(self,name,arg_list = None):
        super(BatterySufficient,self).__init__(name,arg_list)

    def __deepcopy__(self, memodict={}):
        return BatterySufficient(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(selfs,generated_Values,ll_state):
        ll_state.ll_variables["battery"] = 500
