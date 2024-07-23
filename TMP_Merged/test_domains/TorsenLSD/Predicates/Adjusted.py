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
        self.manual_part_positions = {}
        with open(Config.MANUAL_PART_POSITIONS_PKL) as f:
            self.manual_part_positions = pickle.load(f)

    def __deepcopy__(self, memodict={}):
        return Adjusted(copy.deepcopy(self.name), copy.deepcopy(self.arg_list))

    def apply(self, ll_state, generated_values):
        part_name = generated_values['part']
        part_name = self.body_name_dict[part_name]
        self.env = ll_state.simulator.env
        body = self.env.GetKinBody(part_name)

        self.assembled_env = Environment()

        if part_name == 'Base':
            self.assembled_env.Load(Config.FULL_ASSEMBLED_STRUCTURE_PATH)
            body.SetTransform(self.manual_part_positions[part_name])
            for key in self.body_name_dict.keys():
                body_name = self.body_name_dict[key]
                if 'wheel' not in body_name.lower():
                    new_pose = self.get_updated_pose('Base', body_name)
                    body = self.env.GetKinBody(body_name)
                    body.SetTransform(new_pose)
        else:
            if 'Wheel' in part_name:
                self.assembled_env.Load(Config.FULL_ASSEMBLED_STRUCTURE_PATH)
                base_name = 'Base'
            elif part_name[0] == "L":
                side = "left"
                self.assembled_env.Load(Config.LEFT_ASSEMBLED_STRUCTURE_PATH)
                base_name = 'LeftUprightClamp'
            elif part_name[0] == "R":
                side = "right"
                self.assembled_env.Load(Config.RIGHT_ASSEMBLED_STRUCTURE_PATH)
                base_name = 'RightUprightClamp'

            if part_name == base_name:    
                body.SetTransform(self.manual_part_positions[part_name])
            else:
                new_pose = self.get_updated_pose(base_name, part_name)
                # refw_T_base = self.assembled_env.GetKinBody(base_name).GetTransform() # referenceworld_T_base
                # part_T_refw = self.assembled_env.GetKinBody(part_name).GetTransform() # part_T_referenceworld
                # base_T_part = np.matmul(np.linalg.inv(refw_T_base), part_T_refw) # base_T_part
                # w_T_base = self.env.GetKinBody(base_name).GetTransform() # world_T_base
                # new_pose = np.matmul(w_T_base, base_T_part) # world_T_part
                body.SetTransform(new_pose)

    def get_updated_pose(self, base_name, part_name):
        refw_T_base = self.assembled_env.GetKinBody(base_name).GetTransform() # referenceworld_T_base
        part_T_refw = self.assembled_env.GetKinBody(part_name).GetTransform() # part_T_referenceworld
        base_T_part = np.matmul(np.linalg.inv(refw_T_base), part_T_refw) # base_T_part
        w_T_base = self.env.GetKinBody(base_name).GetTransform() # world_T_base
        new_pose = np.matmul(w_T_base, base_T_part) # world_T_part
        return new_pose