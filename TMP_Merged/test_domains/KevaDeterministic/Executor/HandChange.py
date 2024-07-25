import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class HandChange(ArgExecutor):

    def __init__(self,argument_name):
        super(HandChange,self).__init__(argument_name)
    def __deepcopy__(self, memodict={}):
        return HandChange(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        if value in ["left","right"]:
            simulator = low_level_state.simulator
            simulator.robots[other_generated_values["robot"]].activate_arm(value)

    def execute(self,low_level_state,value,other_generated_values):
        if value in ["left", "right"]:
            simulator = low_level_state.simulator
            simulator.robots[other_generated_values["robot"]].activate_arm(value)

