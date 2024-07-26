from src.DataStructures.Predicate import Predicate
from openravepy import *
import copy
import numpy as np
import pickle
import Config

class Adjusted(Predicate):
    def __init__(self, name, arg_list=None):
        super(Adjusted, self).__init__(name, arg_list)
        self.body_name_dict = Config.body_name_dict
        self.clamp_positions = {}
        with open(Config.CLAMP_POSITIONS_PKL) as f:
            self.clamp_positions = pickle.load(f)

    def __deepcopy__(self, memodict={}):
        return Adjusted(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):

        '''
        write the code to adjust the parts to the correct locations here...
        '''

        part_name = generated_values['part']
        env = ll_state.simulator.env

        assembled_env = Environment()
        if part_name[0] == "l":
            side = "left"
            assembled_env.Load(Config.LEFT_ASSEMBLED_STRUCTURE_PATH)
            base_name = 'LeftUprightClamp'
        else:
            side = "right"
            assembled_env.Load(Config.RIGHT_ASSEMBLED_STRUCTURE_PATH)
            base_name = 'RightUprightClamp'
        
        part_name = self.body_name_dict[part_name]
        body = env.GetKinBody(part_name)

        if part_name == base_name:    
            body.SetTransform(self.clamp_positions[part_name])
        else:
            refw_T_base = assembled_env.GetKinBody(base_name).GetTransform() # referenceworld_T_base
            part_T_refw = assembled_env.GetKinBody(part_name).GetTransform() # part_T_referenceworld
            base_T_part = np.matmul(np.linalg.inv(refw_T_base), part_T_refw) # base_T_part
            w_T_base = env.GetKinBody(base_name).GetTransform() # world_T_base
            new_pose = np.matmul(w_T_base, base_T_part) # world_T_part
            body.SetTransform(new_pose)
