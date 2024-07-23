from src.DataStructures.Generator import Generator
import numpy as np
from openravepy import *



class BasePoseAroundTableGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['pose_current']
        super(BasePoseAroundTableGenerator, self).__init__(known_argument_values, required_values)
        self.known_argument_values = known_argument_values
        self.simulator = ll_state.simulator
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        transforms_list = self.generate_poses()
        # current_transform = self.simulator.env.GetRobot('fetch').GetTransform()
        # transforms_list.insert(0,[current_transform])
        for t in transforms_list:
            yield t

    def get_next(self,flag):
        return self.generate_function_state.next()


    def generate_poses(self):
        # env = self.simulator.env
        # current_transform = self.simulator.env.GetRobot('fetch').GetLink("base_link").GetTransform()
        # l = [[0,0,0],
        #      [1.7,0,3.14159],
        #      [0.85,0.85,4.71239],
        #      [0.85,-0.85,1.5708]]
        #
        # current_x = current_transform[0,3]
        # current_y = current_transform[1,3]
        # c_index = None
        #
        # for i in range(len(l)):
        #     if abs(l[i][0]- current_x) < 1e-3 and abs(l[i][1]-current_y) < 1e-3:
        #         c_index = i
        #
        # c_transform = l[c_index]
        # l.remove(c_transform)
        #
        # np.random.shuffle(l)
        # l.insert(0,c_transform)
        # generated_poses = l
        while True:
            if "tabel1" in self.known_argument_values["location"]:
                return [[0,0,0]]
            elif "table2" in self.known_argument_values["location"]:
                return [[0,0,1.5708]]
            elif "table3" in self.known_argument_values["location"]:
                return [[0,0,4.71239]]
        return generated_poses

