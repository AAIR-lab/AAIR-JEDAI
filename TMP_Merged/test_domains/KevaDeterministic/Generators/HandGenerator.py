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
        required_values = ['robot', 'obj']

        super(HandGenerator, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('obj')
        self.location = known_argument_values.get('location')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        location = self.location
        # if self.object_name in ["plank3","plank4","plank7","plank8","plank11","plank12","plank15","plank16"]:
        #     active_hands = ["left"]
        # else:
        #     active_hands = ["right"]
        if self.known_argument_values["gripper"] == "left":
            active_hands = ["left"]
        elif self.known_argument_values["gripper"] == "right":
            active_hands = ["right"]
        else:
            active_hands = ["left","right"]
        for gt in active_hands:
            # self.simulator.robots[self.known_argument_values["robot"]].activate_arm(gt)
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()