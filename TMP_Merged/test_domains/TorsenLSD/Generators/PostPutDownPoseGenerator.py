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

class PostPutDownPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot']

        super(PostPutDownPoseGenerator, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.putdown_pose = known_argument_values.get('putdown_pose')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        pose_list =[]
        env = self.simulator.env
        robot = env.GetRobot(self.known_argument_values["robot"])

        if robot.GetName() == 'fetch':
            post_putdown_offset = 0.06
            postputdown_gripper_transform = np.identity(4)
            postputdown_gripper_transform[0][3] -= post_putdown_offset

            postputdown_gripper_pose = np.matmul(self.putdown_pose, postputdown_gripper_transform)
        else:
            post_putdown_offset = 0.04
            postputdown_gripper_transform = np.identity(4)
            postputdown_gripper_transform[2][3] -= post_putdown_offset

            postputdown_gripper_pose = np.matmul(self.putdown_pose, postputdown_gripper_transform)

        pose_list.append(postputdown_gripper_pose)
        return pose_list