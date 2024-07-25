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

class PreGraspPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj']

        super(PreGraspPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('obj')
        self.grasp_region = known_argument_values.get('region')
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
        pre_grasp_offset = 0.03
        # pre grasp pose is always offset in z axis in gripper frame
        world_T_gripper = self.grasp_pose.copy()
        gripper_T_gripper_new = np.eye(4)
        gripper_T_gripper_new[2][3] -= pre_grasp_offset

        world_T_gripper_new = np.matmul(world_T_gripper, gripper_T_gripper_new)
        pose_list = []
        ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(world_T_gripper_new, check_collisions=True)
        if len(ik_sols)>0:
            pose_list.append(world_T_gripper_new)
        return pose_list