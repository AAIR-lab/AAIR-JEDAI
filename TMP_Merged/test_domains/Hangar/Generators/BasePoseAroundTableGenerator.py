from src.DataStructures.Generator import Generator
import numpy as np
from openravepy import *
import math


class BasePoseAroundTableGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['cbpose']
        super(BasePoseAroundTableGenerator, self).__init__(known_argument_values, required_values)
        self.known_argument_values = known_argument_values
        self.simulator = ll_state.simulator
        self.generate_function_state = self.generate_function()
        self.number_of_values = 4

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        transforms_list = self.generate_poses()
        # current_transform = self.simulator.env.GetRobot('fetch').GetTransform()
        # transforms_list.insert(0,[current_transform])
        for t in transforms_list:
            yield t

    def get_next(self,flag):
        return self.generate_function_state.next()


    def generate_poses(self):
        env = self.simulator.env

        rotation_90 = quatFromAxisAngle([0, 0, 1.5708])
        rotation_180 = quatFromAxisAngle([0, 0, 3.14159])
        rotation_270 = quatFromAxisAngle([0, 0, 4.71239])

        # distance_robot_table = poseFromMatrix(env.GetKinBody('table6').GetTransform())[-3] - poseFromMatrix(self.simulator.robot_init_pose)[-3]
        distance_robot_table = 1.0
        translation_right_of_table = [distance_robot_table*2,0,0]
        translation_behind_table = [distance_robot_table,distance_robot_table,0]
        translation_front_of_table = [distance_robot_table,-1*distance_robot_table,0]


        # global inital_transform_stored
        # global stored_transform
        # if not BasePoseAroundTableGenerator.inital_transform_stored:
        #     BasePoseAroundTableGenerator.stored_transform= robot.GetTransform()
        #     BasePoseAroundTableGenerator.inital_transform_stored = True
        current_transform = self.simulator.env.GetRobot('fetch').GetLink("base_link").GetTransform()

        # l = [ [0,0,0] , translation_right_of_table[:2] + [3.14159], translation_behind_table[:2] + [4.71239], translation_front_of_table[:2] + [1.5708] ]

        l = [[0,0,0],
             [distance_robot_table,-distance_robot_table,math.pi / 2.0],
             [2 * distance_robot_table, 0, math.pi],
             [distance_robot_table,distance_robot_table,3 * math.pi/2.0]]

        current_x = current_transform[0,3]
        current_y = current_transform[1,3]

        # import IPython
        # IPython.embed()

        c_index = None

        for i in range(len(l)):
            if abs(l[i][0]- current_x) < 1e-3 and abs(l[i][1]-current_y) < 1e-3:
                c_index = i

        # c_transform = l[c_index]
        # l.remove(c_transform)
        #
        # # np.random.shuffle(l)
        # l.insert(0,c_transform)
        # generated_poses = l

        transform_list = []
        while len(transform_list) < len(l):
            transform_list.append(l[c_index])
            c_index = (c_index + 1)%len(l)


        # T_robot_left_of_table = self.simulator.robot_init_pose
        # T_robot_right_of_table = matrixFromPose(np.concatenate((rotation_180,translation_right_of_table), axis=0))
        # T_robot_behind_table = matrixFromPose(np.concatenate((rotation_270,translation_behind_table), axis=0))
        # T_robot_front_of_table = matrixFromPose(np.concatenate((rotation_90, translation_front_of_table), axis=0))



        #TODO The below is a list of lists only to get into the correct flow in OpenRaveSimulator.py line 258, to differentiate between base and arm transforms, needs a change
        # generated_poses = [[T_robot_left_of_table], [T_robot_right_of_table], [T_robot_front_of_table], [T_robot_behind_table],   ]
        # generated_poses = [[T_robot_left_of_table]]
        # np.random.shuffle(generated_poses)
        # generated_poses.insert(0,current_transform)


        return transform_list

