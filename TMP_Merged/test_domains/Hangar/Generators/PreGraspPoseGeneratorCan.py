import copy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import time

class PreGraspPoseGeneratorCan(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["gpose"]

        super(PreGraspPoseGeneratorCan, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.gpose = known_argument_values["gpose"]
        self.offset = 0.10
        self._compute_pose_list()
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):
        env = self.simulator.env
        # robot = env.GetRobot('fetch')
        # cur_dof = robot.GetActiveDOFValues()
        # manipulator = robot.GetActiveManipulator()
        # manip = robot.SetActiveManipulator(manipulator)
        # with env:  # lock environment
        #     Tgoal = self.gpose
        #     sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
        #     if sol is not None:
        #         robot.SetActiveDOFValues(sol)
        # gf = robot.GetLink('gripper_link').GetTransform()
        t0 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
        t1 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, -self.offset, 0, 0)
        tpgp_wrt_gp = t0.dot(t1)
        pgp = self.gpose.dot(t1)
        if OpenraveUtils.has_ik_to(self.simulator.env, self.simulator.env.GetRobots()[0], pgp):
            return [pgp]
        else:
            return []
