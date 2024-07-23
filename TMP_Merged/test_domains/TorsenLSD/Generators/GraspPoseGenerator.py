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
        required_values = ['robot', 'part', 'gripper']
        super(GraspPoseGenerator, self).__init__(known_argument_values, required_values)

        self.body_name_dict = Config.body_name_dict
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.part_name = known_argument_values.get('part')
        self.body_transform = self.simulator.env.GetKinBody(self.body_name_dict[self.part_name]).GetTransform()
        self.generate_function_state = self.generate_function()


    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()


    def _compute_pose_list(self):

        part_name = self.body_name_dict[self.part_name]
        body = self.simulator.env.GetKinBody(part_name)
        body_dim = body.ComputeAABB().extents() * 2
        self.current_body_transform = body.GetTransform()

        if 'Housing' in part_name:
            if 'Left' in part_name:
                grasp_poses = self.get_housing_poses('left')
            else:
                grasp_poses = self.get_housing_poses('right')
        elif 'Axle' in part_name:
            grasp_poses = self.get_axle_poses()
        elif 'Drive' in part_name:
            grasp_poses = self.get_drive_poses()
        elif 'Upright' in part_name:
            grasp_poses = self.get_upright_poses()
        elif 'WormSpur' in part_name:
            grasp_poses = self.get_wormspur_poses()
        elif 'RingGear' in part_name:
            grasp_poses = self.get_ring_gear_poses()
        elif 'Shaft' in part_name:
            grasp_poses = self.get_shaft_poses()
        elif 'Wheel' in part_name:
            grasp_poses = self.get_wheel_poses()
        elif 'Base' in part_name:
            grasp_poses = self.get_base_poses()
        else:
            print("unknown part")

        return grasp_poses

    def get_housing_poses(self, side):
        pose_list = []

        if side.lower() == 'left':
            # grasping from protruding clamps where spur gears fit in
            z_rotation_matrix = self.rotate_z(np.pi)
            x_rotation_matrix = self.rotate_x(0)
            y_rotation_matrix = self.rotate_y(0)
            translate_matrix = self.translate(-0.23,0.053,-0.05)
            transformations = [z_rotation_matrix, translate_matrix]
        else:
            z_rotation_matrix = self.rotate_z(np.pi)
            x_rotation_matrix = self.rotate_x(0)
            y_rotation_matrix = self.rotate_y(0)
            translate_matrix = self.translate(-0.25,0.053,-0.05)
            transformations = [z_rotation_matrix, translate_matrix]
            
        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)
        
        pose_list.append(grasp_pose)
        return pose_list

    def get_axle_poses(self):
        pose_list = []

        z_rotation_matrix = self.rotate_z(np.pi/2)
        x_rotation_matrix = self.rotate_x(np.pi/2)
        y_rotation_matrix = self.rotate_y(0)
        translate_matrix = self.translate(0,-0.02,-0.12)
        transformations = [x_rotation_matrix, z_rotation_matrix, translate_matrix]

        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)

        pose_list.append(grasp_pose)
        return pose_list

    def get_wormspur_poses(self):
        pose_list = []

        z_rotation_matrix = self.rotate_z(0)
        x_rotation_matrix = self.rotate_x(0)
        y_rotation_matrix = self.rotate_y(-np.pi/2)
        translate_matrix = self.translate(0,0,-0.13)
        transformations = [y_rotation_matrix, z_rotation_matrix, translate_matrix]

        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)

        pose_list.append(grasp_pose)
        return pose_list

    def get_upright_poses(self):
        pose_list = []
        
        z_rotation_matrix = self.rotate_z(-np.pi/2)
        x_rotation_matrix = self.rotate_x(0)
        y_rotation_matrix = self.rotate_y(0)
        translate_matrix = self.translate(-0.25,-0.012,0)
        transformations = [z_rotation_matrix, translate_matrix]
        
        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)
        
        pose_list.append(grasp_pose)
        return pose_list

    def get_ring_gear_poses(self):
        pose_list = []

        z_rotation_matrix = self.rotate_z(0)
        x_rotation_matrix = self.rotate_x(np.pi/2)
        y_rotation_matrix = self.rotate_y(np.pi/2)
        translate_matrix = self.translate(-0.27,0,0)
        transformations = [y_rotation_matrix,x_rotation_matrix,translate_matrix]
        
        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)
        
        pose_list.append(grasp_pose)
        return pose_list

    def get_drive_poses(self):
        pose_list = []

        z_rotation_matrix = self.rotate_z(0)
        x_rotation_matrix = self.rotate_x(np.pi/2)
        y_rotation_matrix = self.rotate_y(np.pi/2)
        translate_matrix = self.translate(-0.19,-0.0129,0)
        transformations = [y_rotation_matrix,x_rotation_matrix, translate_matrix]
            
        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)
        
        pose_list.append(grasp_pose)
        return pose_list

    def get_shaft_poses(self):
        pose_list = []

        z_rotation_matrix = self.rotate_z(np.pi/2)
        x_rotation_matrix = self.rotate_x(0)
        y_rotation_matrix = self.rotate_y(-np.pi/2)
        translate_matrix = self.translate(0,-0.05,-0.12)
        transformations = [y_rotation_matrix, z_rotation_matrix, translate_matrix]
            
        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)

        pose_list.append(grasp_pose)
        return pose_list

    def get_wheel_poses(self):
        pose_list = []

        z_rotation_matrix = self.rotate_z(0)
        x_rotation_matrix = self.rotate_x(0)
        y_rotation_matrix = self.rotate_y(np.pi)
        translate_matrix = self.translate(0,0,-0.16)
        transformations = [y_rotation_matrix, z_rotation_matrix, translate_matrix]

        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)

        pose_list.append(grasp_pose)
        return pose_list

    def get_base_poses(self):
        pose_list = []

        com = self.simulator.env.GetKinBody('Base').ComputeAABB().pos()
        t = self.simulator.env.GetKinBody('Base').GetTransform()
        x_diff = abs(com[0] - t[0,3])
        y_diff = abs(com[1] - t[1,3])
        z_diff = abs(com[2] - t[2,3])

        z_rotation_matrix = self.rotate_z(0)
        x_rotation_matrix = self.rotate_x(0)
        y_rotation_matrix = self.rotate_y(np.pi/2)
        translate_matrix_com = self.translate(x_diff,y_diff,z_diff)
        translate_matrix = self.translate(-0.22,0,0)
        transformations = [translate_matrix_com, y_rotation_matrix, z_rotation_matrix, translate_matrix]
            
        grasp_pose = self.current_body_transform
        for transform in transformations:
            grasp_pose = np.matmul(grasp_pose, transform)

        pose_list.append(grasp_pose)
        return pose_list

    def rotate_z(self,rot_angle):
        # rotate around z axis
        rotation_matrix = np.identity(4)
        rotation_matrix[0][0] = math.cos(rot_angle)
        rotation_matrix[0][1] = -(math.sin(rot_angle))
        rotation_matrix[1][0] = math.sin(rot_angle)
        rotation_matrix[1][1] = math.cos(rot_angle)
        return rotation_matrix

    def rotate_y(self,rot_angle):
        # rotate around y axis
        rotation_matrix = np.identity(4)
        rotation_matrix[0][0] = math.cos(rot_angle)
        rotation_matrix[0][2] = math.sin(rot_angle)
        rotation_matrix[2][0] = -math.sin(rot_angle)
        rotation_matrix[2][2] = math.cos(rot_angle)
        return rotation_matrix

    def rotate_x(self,rot_angle):
        # rotate around x axis
        rotation_matrix = np.identity(4)
        rotation_matrix[1][1] = math.cos(rot_angle)
        rotation_matrix[1][2] = -(math.sin(rot_angle))
        rotation_matrix[2][1] = math.sin(rot_angle)
        rotation_matrix[2][2] = math.cos(rot_angle)
        return rotation_matrix

    def translate(self,x, y, z):
        # rotate around x axis
        rotation_matrix = np.identity(4)
        rotation_matrix[0][3] = x
        rotation_matrix[1][3] = y
        rotation_matrix[2][3] = z
        return rotation_matrix
