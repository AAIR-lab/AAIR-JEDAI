from src.DataStructures.Generator import Generator
import numpy as np
from openravepy import *
import math
import random


class BasePoseAroundTableGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['cbpose']
        super(BasePoseAroundTableGenerator, self).__init__(known_argument_values, required_values)
        self.known_argument_values = known_argument_values
        self.simulator = ll_state.simulator
        self.table_name = self.known_argument_values['location']
        self.robot_name = 'fetch'
        self.generate_function_state = self.generate_function()
        self.number_of_values = 4

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        transforms_list = self.generate_poses()
        for t in transforms_list:
            yield t

    def get_next(self,flag):
        return self.generate_function_state.next()


    def generate_poses(self):
        env = self.simulator.env
        diff = 0.2
        if self.table_name.startswith('table'):
            table_extents = self.simulator.env.GetKinBody(self.table_name).ComputeAABB().extents()
            table_x, table_y, _ = self.simulator.env.GetKinBody(self.table_name).ComputeAABB().pos()

            pose_list = [[table_x-table_extents[0]-diff,table_y,0],
             [table_x,table_y-table_extents[1]-diff,math.pi / 2.0],
             [table_x+table_extents[0]+diff,table_y, math.pi],
             [table_x,table_y+table_extents[1]+diff,3 * math.pi/2.0]]
        else:
            # Case for fixed counter area
            # pose_list = [[2.8,16.5,0],
            #      [2.8,16.7,0],]
            pose_list = [[0.8,0.5,0],
                 [0.8,0.7,0],]
        # import IPython
        # IPython.embed()
        random.shuffle(pose_list)
        return pose_list

