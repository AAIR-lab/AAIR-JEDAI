import copy
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *


class GraspPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ["obj"]

        super(GraspPoseGenerator, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.object_name = known_argument_values["obj"]
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        self.number_of_values = 4


    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

    def generate_function_err_free(self):
        for gt in self._compute_pose_list(False):
            yield gt
        raise StopIteration

    def generate_function_err(self):
        for gt in self._compute_pose_list(True):
            yield gt
        raise StopIteration

    def get_next(self,flag):
        if not flag["flag"]:
            return self.generate_function_state_err.next()
        else:
            return self.generate_function_state_err_free.next()

    def _compute_pose_list(self,remove_bodies):
        object_name = self.object_name
        env = self.simulator.env
        base_width, base_length, cylinder_height = self.simulator.get_object_link_dimensions(object_name=object_name, link_name='base')

        orig_transform = self.simulator.get_transform_matrix(object_name)
        self.simulator.set_transform_matrix(object_name, np.identity(4))
        self.simulator.set_transform_matrix(object_name, orig_transform)

        num_grasps = 4

        height_offset = -0.1
        dist_offset = 0.04
        if cylinder_height == 0:
            cylinder_height = 0.20
        if remove_bodies:
            name_to_object_and_transform = self.simulator.remove_all_removable_bodies(self.simulator.env)

        robot = env.GetRobots()[0]
        wrist_roll_pose = robot.GetLink('wrist_roll_link').GetTransform()
        gripper_pose = robot.GetLink('gripper_link').GetTransform()
        wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)

        j = 0
        pose_list = []
        for i in range(num_grasps):
            rot_ang = (i * (2 * np.pi) / num_grasps)
            t2 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, -dist_offset, 0, cylinder_height + height_offset)

            r2 = self.simulator.get_matrix_from_axis_angle(0, 0, rot_ang)
            t3 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
            grasp_pose = t3.dot(t2).dot(r2)
            grasp_pose = np.matmul(orig_transform,grasp_pose)
            grasp_pose = np.matmul(grasp_pose, wrist_pose_wrt_gripper)

            if len(self.simulator.robots[self.robot_name].get_ik_solutions(grasp_pose, True)) > 0:
                pose_list.append(grasp_pose)
        if remove_bodies:
            with self.simulator.env:
                for body_name in name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])
        return pose_list
