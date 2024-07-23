import openravepy as orpy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random
from trac_ik_python.trac_ik import IK
import Config

class GraspPoseGeneratorKeva(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['robot', 'obj', 'region']

        super(GraspPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.plank_name = known_argument_values.get('obj')
        if Config.LOOPED_RUNS:
            self.plank_name = Config.correct_plank_name(self.plank_name)
        self.grasp_region = known_argument_values.get('region')
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
        robot = env.GetRobots()[0]
        plank = env.GetKinBody(self.plank_name)

        world_T_plank = plank.GetTransform()

        world_T_robot = robot.GetTransform()
        robot_T_world = np.linalg.inv(world_T_robot)

        plank_T_gripper = np.eye(4)
        plank_T_gripper[1,3]= -0.135 #gripper_offset

        t1 = orpy.matrixFromAxisAngle([ 0, -np.pi/2, 0])
        t2 = orpy.matrixFromAxisAngle([-np.pi/2, 0, 0])

        plank_T_gripper = np.matmul(np.matmul(plank_T_gripper,t1),t2)

        robot_T_gripper = np.matmul(np.matmul(robot_T_world,world_T_plank),plank_T_gripper)

        # openrave requires gripper in world frame
        world_T_gripper = np.matmul(world_T_robot,robot_T_gripper)
        pose_list =[]
        ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(world_T_gripper, check_collisions=True)
        if len(ik_sols)>0:
            pose_list.append(world_T_gripper)
        return pose_list