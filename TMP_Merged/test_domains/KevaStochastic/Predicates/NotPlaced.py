from src.DataStructures.Predicate import Predicate
from openravepy import *
import Config
import copy
import numpy as np


class NotPlaced(Predicate):
    def __init__(self, name, arg_list=None):
        super(NotPlaced, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return NotPlaced(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        current_hl_state = generated_values["current_hl_state"].getTrueProps()
        
        plank = generated_values["plank"]
        location = generated_values["location"]
        
        assert "(handempty)" in current_hl_state
        assert "(human_placed %s %s)" % (plank, location) in current_hl_state
        assert "(free %s)" % (location) not in current_hl_state
        assert "(orientation %s sideways)" % (plank) in current_hl_state
        assert "(clearplank %s)" % (plank) in current_hl_state
        
        # Set it to the default position in the environment.
        transform = np.asarray([[ 1,      0,      0,      0,    ],
            [ 0,      1,      0,      0.2   ],
            [ 0,      0,      1,     -0.4145],
            [ 0,      0,      0,      1    ]])
        
        p = ll_state.simulator.env.GetKinBody(plank)
        p.SetTransform(transform)

