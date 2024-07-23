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

class PostGraspPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'part']

        super(PostGraspPoseGenerator, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.grasp_pose = known_argument_values.get('grasp_pose')
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        pre_grasp_offset = 0.04
        pose_list =[]
        env = self.simulator.env
        robot = env.GetRobot(self.known_argument_values["robot"])

        # fetch
        if robot.GetName() == 'fetch':
            post_grasp_offset = 0.07
            post_grasp_matrix = np.identity(4)
            post_grasp_matrix[0][3] -= post_grasp_offset
            postpickup_gripper_pose = np.matmul(self.grasp_pose, post_grasp_matrix)
        else:
            post_grasp_offset = 0.04
            postpickup_gripper_pose = self.grasp_pose.copy()
            postpickup_gripper_pose[2][3] += post_grasp_offset        

        pose_list.append(postpickup_gripper_pose)
        return pose_list