from src.DataStructures.Predicate import Predicate
import copy
import numpy as np

class Crushed(Predicate):
    def __init__(self, name, arg_list=None):
        super(Crushed, self).__init__(name, arg_list)
        self.ar

    def __deepcopy__(self, memodict={}):
        return Crushed(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        next_hl_state = generated_values["current_hl_state"].getTrueProps()
        for prop in next_hl_state:
            if generated_values["obj"] in prop and "crushed" in prop:
                t = ll_state.simulator.env.GetKinBody(generated_values["obj"]).GetTransform()
                t[2,3] = 0
                ll_state.simulator.env.GetKinBody(generated_values["obj"]).SetTransform(t)
