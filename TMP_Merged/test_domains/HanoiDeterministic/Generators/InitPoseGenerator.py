from src.DataStructures.Generator import Generator
import numpy as np

import pickle

class InitPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(InitPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR
        self.init_pose = np.array([0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565 ])


    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            yield self.init_pose

    def get_next(self,flag):
        return self.generate_function_state.next()

