from src.DataStructures.Predicate import Predicate
import openravepy as orpy
import Config
import copy
import numpy as np


class Gravity(Predicate):
    def __init__(self, name, arg_list=None):
        super(Gravity, self).__init__(name, arg_list)

    def __deepcopy__(self, memodict={}):
        return Gravity(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        plank = generated_values["obj"]
        reference_env = orpy.Environment()
        reference_env.Load(Config.REFERENCE_STRUCTURE_PATH)
        if Config.LOOPED_RUNS:
            plank = Config.correct_plank_name(plank)
        world_Tref_root_plank = reference_env.GetKinBody("plank1").GetTransform()
        root_plank_Tref_world = np.linalg.inv(world_Tref_root_plank)
        world_Tref_curr_plank = reference_env.GetKinBody(plank).GetTransform()
        root_plank_T_curr_plank = np.matmul(root_plank_Tref_world,world_Tref_curr_plank)

        # Curr env
        world_T_root_plank = ll_state.simulator.env.GetKinBody("plank1").GetTransform()
        world_T_curr_plank = np.matmul(world_T_root_plank,root_plank_T_curr_plank)


        p = ll_state.simulator.env.GetKinBody(plank)
        p.SetTransform(world_T_curr_plank)