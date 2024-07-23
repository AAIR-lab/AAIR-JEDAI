import random
from src.DataStructures.Generator import Generator
import src.OpenraveUtils as OpenraveUtils
import numpy as np
from openravepy import *
import math
import copy
import Config
from trac_ik_python.trac_ik import IK


class PutDownPoseGenerator(Generator):

    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['robot', 'part']
        super(PutDownPoseGenerator, self).__init__(known_argument_values, required_values)
        
        self.body_name_dict = Config.body_name_dict
        self.simulator = ll_state.simulator
        self.env = self.simulator.env
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()
        self.table_name = 'table60'
        self.part_name = known_argument_values.get('part')
        self.robot = self.simulator.env.GetRobot(known_argument_values.get('robot'))

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_put_down_pose_list(self):

        self.robot = self.env.GetRobot(self.known_argument_values["robot"])
        part_name = self.body_name_dict[self.part_name]
        self.reference_env = Environment()

        if self.robot.GetName() == 'fetch':
            self.reference_env.Load(Config.FETCH_REFERENCE_STRUCTURE_PATH)
            reference_root_part = 'table60'
            putdown_gripper_pose = self.get_gripper_pose('base_link', reference_root_part, part_name)
            robot_world_transform = self.robot.GetLink('base_link').GetTransform()
            putdown_gripper_pose = np.matmul(robot_world_transform, putdown_gripper_pose)
        else: # yumi
            if 'wheel' in self.part_name.lower():
                self.reference_env.Load(Config.WHEELS_REFERENCE_PATH)
                reference_root_part = 'Base'
            if self.part_name[0] == "l":
                self.reference_env.Load(Config.LEFT_REFERENCE_STRUCTURE_PATH)
                reference_root_part = 'LeftUprightClamp'
            else:
                self.reference_env.Load(Config.RIGHT_REFERENCE_STRUCTURE_PATH)
                reference_root_part = 'RightUprightClamp'
            putdown_gripper_pose = self.get_gripper_pose('world', reference_root_part, part_name)

        return [putdown_gripper_pose]

    def get_gripper_pose(self, base_link_name, reference_root_part, part_name):
        robot_world_transform = self.robot.GetLink(base_link_name).GetTransform() # w_T_r
        gripper_base_link = self.robot.GetActiveManipulator().GetEndEffector()
        gripper_base_transform = gripper_base_link.GetTransform() # w_T_g
        grabbed_object_pose = self.env.GetKinBody(part_name).GetTransform() # w_T_obj
        object_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_base_transform), grabbed_object_pose) # g_T_obj

        reference_root_transform = self.reference_env.GetKinBody(reference_root_part).GetTransform() # w_T_base
        reference_part_transform = self.reference_env.GetKinBody(part_name).GetTransform() # w_T_obj
        reference_part_wrt_root = np.matmul(np.linalg.inv(reference_root_transform),
                                                            reference_part_transform) # base_T_obj
        root_transform = self.env.GetKinBody(reference_root_part).GetTransform() # w_T_base

        updated_object_pose = np.matmul(root_transform, reference_part_wrt_root) # w_T_obj : new pose of object

        object_pose_wrt_robot = np.matmul(np.linalg.inv(robot_world_transform), updated_object_pose) # r_T_obj
        putdown_gripper_pose = np.matmul(object_pose_wrt_robot, np.linalg.inv(object_pose_wrt_gripper)) # r_T_g

        return putdown_gripper_pose