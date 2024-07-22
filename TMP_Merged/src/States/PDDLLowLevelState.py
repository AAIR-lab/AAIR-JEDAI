from src.States.PDDLState import PDDLState
import copy
import Config


class PDDLLowLevelState(PDDLState):
    def __init__(self,trueSet = None, falseSet = None):
        super(PDDLLowLevelState,self).__init__(trueSet,falseSet)
        self.history = []
        self.ll_history = []
        self.ll_variables = {}
        self.values = self.getAllProps()


    def set_values(self,values):
        self.reset_state()
        self.addProps(values)

    def __deepcopy__(self, memodict={}):
        new_pddl_low_level_state = PDDLLowLevelState()
        new_pddl_low_level_state.addTrueProps(self.getTrueProps())
        # new_pddl_low_level_state.(self.getFalseProps())
        # new_pddl_low_level_state.self.getObjDict())
        new_pddl_low_level_state.history = []
        new_pddl_low_level_state.ll_history = []
        new_pddl_low_level_state.ll_variables = copy.deepcopy(new_pddl_low_level_state.ll_variables)
        return new_pddl_low_level_state


    def apply(self,argument,other_generated_values):
        # Code to update the state based on the low-level plan
        self.history.append(self.values)
        self.ll_history.append(self.ll_variables)
        argument.apply(self, other_generated_values)
        pass

    def execute(self,argument,other_generated_values):
        argument.execute(self,other_generated_values)
        pass

    def apply_effect(self,effect,generated_values):
        pass

    def rollback(self):
        pass

    def sync_simulator(self,old_state = None):
        pass

    def get_values_from_env(self,openrave_env = None):
        return self.getTrueProps()

