from openravepy import *
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random
import Config

class PostPutDownPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj']

        super(PostPutDownPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('obj')
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
        post_putdown_offset = 0.03
        # post putdown pose is always offset in z axis in gripper frame
        world_T_gripper = self.putdown_pose.copy()
        gripper_T_gripper_new = np.eye(4)
        gripper_T_gripper_new[2][3] -= post_putdown_offset

        world_T_gripper_new = np.matmul(world_T_gripper, gripper_T_gripper_new)
        pose_list = []
        ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(world_T_gripper_new, check_collisions=True)
        if len(ik_sols)>0:
            pose_list.append(world_T_gripper_new)
        return pose_list