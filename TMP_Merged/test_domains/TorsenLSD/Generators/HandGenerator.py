from openravepy import *
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random
from trac_ik_python.trac_ik import IK
import Config

class HandGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'part', 'gripper']

        super(HandGenerator, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('part')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        if self.known_argument_values["robot"] == "yumi":
            if self.object_name[0] == "l":
                # self.simulator.robots[self.known_argument_values["robot"]].activate_arm("left")
                yield "left"
            else:
                # self.simulator.robots[self.known_argument_values["robot"]].activate_arm("right")
                yield "right"
        else:
            for gt in ["main"]:
                yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()