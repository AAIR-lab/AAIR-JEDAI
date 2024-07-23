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

class GraspPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'domino', 'gripper']
        super(GraspPoseGenerator, self).__init__(known_argument_values, required_values)

        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.part_name = known_argument_values.get('domino')
        self.generate_function_state = self.generate_function()


    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()


    def _compute_pose_list(self):

        '''
        make sure the names are matching the names in the pddl.
        :return:
        '''
        object_pose = self.simulator.env.GetKinBody(self.part_name).GetTransform()
        x_offset = 0.15
        y_rotation = -math.pi/2
        t2 = self.simulator.get_matrix_from_pose(1,0,0,0,x_offset,0,0)
        r2 = self.simulator.get_matrix_from_axis_angle(0,0,0)
        r3 = self.simulator.get_matrix_from_axis_angle(0,y_rotation,0)
        grasp_pose = t2.dot(r2).dot(r3)
        grasp_pose = object_pose.dot(grasp_pose)
        r = self.simulator.get_matrix_from_axis_angle(0, 0, math.pi / 2.0)
        grasp_pose = grasp_pose.dot(r)
        return [grasp_pose]