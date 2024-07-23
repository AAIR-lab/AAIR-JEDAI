from src.DataStructures.Predicate import Predicate
from openravepy import *
import Config
import copy
import numpy as np


class Placed(Predicate):
    def __init__(self, name, arg_list=None):
        super(Placed, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return Placed(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        next_hl_state = generated_values["next_hl_state"].getTrueProps()
        plank = generated_values["plank"]
        if Config.LOOPED_RUNS:
            plank = Config.correct_plank_name(plank)
        rel_pose = matrixFromPose([ 0.5, -0.5, -0.5,  0.5, -0.05884+0.003875,0.,1.5*(2*0.003875)+0.01162+0.0001])
        for prop in next_hl_state:
            if "human_placed" in prop:
                if "location1" in prop:
                    central_plank = ll_state.simulator.env.GetKinBody('left_station_base').GetTransform()
                elif "location2" in prop:
                    central_plank = ll_state.simulator.env.GetKinBody('right_station_base').GetTransform()
                break
            # if "human_placed" in prop: and plank in prop and "location1" in prop:
            #     
            #     t = [[0.0,  0.0, -1.0,  0.4916909], \
            #          [1.0, 0.0, 0.0, 0.19588947], \
            #          [0.0, -1.0, 0.0, 0.10786212], \
            #          [0.0, 0.0, 0.0, 1.0]]  # change it to transform of location1
            #     break
            # elif "human_placed" in prop and plank in prop and "location2" in prop:
            #     t = [[0.0,  0.0, -1.0,  0.4916909], \
            #          [1.0, 0.0, 0.0, 0.34088948], \
            #          [0.0, -1.0, 0.0, 0.10786212], \
            #          [0.0, 0.0, 0.0, 1.0]]  # change it to transform of location1
            #     break
        p = ll_state.simulator.env.GetKinBody(plank)
        t = np.matmul(central_plank,rel_pose)
        p.SetTransform(t)
