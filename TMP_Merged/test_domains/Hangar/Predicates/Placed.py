from src.DataStructures.Predicate import Predicate
import copy
import numpy as np

class Placed(Predicate):
    def __init__(self, name, arg_list=None):
        super(Placed, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return Placed(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, generated_values, ll_state):
        next_hl_state = generated_values["next_hl_state"].getTrueProps()
        plank = generated_values["plank"]
        for prop in next_hl_state:
            if "human_placed" in prop and plank in prop and "location1" in prop:
                t = np.eye(4)  # change it to transform of location1
                break
            elif "human_placed" in prop and plank in prop and "location2" in prop:
                t = np.eye(4)  # change it to transform of location1
                break
        p = ll_state.simulator.GetKinBody(plank)
        p.SetTransform()
