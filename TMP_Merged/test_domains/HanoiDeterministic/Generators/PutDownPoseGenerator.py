from openravepy import *
import numpy as np
import random
import src.OpenraveUtils as OpenraveUtils
from src.DataStructures.Generator import Generator
from src.Simulators.OpenRaveSimulator import *
import math
import random

class PutDownPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['object1', 'object3']

        super(PutDownPoseGenerator, self).__init__(known_argument_values, required_values)
        self.ll_state = ll_state
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.object_name = known_argument_values.get('object3')
        self.grasped_box_name = known_argument_values.get('object1')
        # self.grasp_region = known_argument_values.get('group')[0]
        self.generate_function_state = self.generate_function()

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_pose_list():
            yield gt

    def get_next(self, flag):
        return self.generate_function_state.next()

    def _compute_pose_list(self):

        # ToDo: Transforms from robot base
        length, breadth, height = self.simulator.get_obj_name_box_extents(object_name=self.grasped_box_name)
        grasp_num = 10
        region_length = length/3.0
        gripper_offset = 0.04 + 0.021
        rot_angle = math.pi / 2.0

        # _, region = self.grasp_region.split('_')
        offset_mean = (region_length * 2) - (2*region_length)

        # grasp_offsets = []
        # for i in range(grasp_num):
        #     grasp_offsets.append(random.uniform(offset_mean-region_length/2.0, offset_mean+region_length/2.0))

        env = self.simulator.env
        if "loc" in self.object_name:
            print(self.object_name)
            loc_no = self.object_name.split("_")[-1]
            print(self.object_name, loc_no)
            if loc_no == "i":
                y = -0.25
            elif loc_no == "ii":
                y = 0
            elif loc_no == "iii":
                y = 0.25
            x = 0.7
            body_trans = np.identity(4)
            body_trans[0][3] = x
            body_trans[1][3] = y
            body_trans[2][3] = 0.603
            # print("Loc")
        else:
            body = env.GetKinBody(self.object_name)
            body_trans = body.GetTransform()
            body_trans[2][3] += 0.020001
        print(body_trans)
        robot = env.GetRobots()[0]
        base_link = robot.GetLinks()[0]
        base_trans = base_link.GetTransform()
        wrist_roll_pose = robot.GetLink('wrist_roll_link').GetTransform()
        gripper_pose = robot.GetLink('gripper_link').GetTransform()

        manipulator = robot.GetActiveManipulator()
        manip = robot.SetActiveManipulator(manipulator)  # set the manipulator to leftarm
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        wrist_pose_wrt_gripper = np.matmul(np.linalg.inv(gripper_pose), wrist_roll_pose)


        rotation_matrix = np.identity(4)
        rotation_matrix[0][0] = math.cos(rot_angle)
        rotation_matrix[0][2] = math.sin(rot_angle)
        rotation_matrix[2][0] = -(math.sin(rot_angle))
        rotation_matrix[2][2] = math.cos(rot_angle)

        rotated_pose = np.matmul(body_trans, rotation_matrix)

        # with env:  # lock environment
        #     Tgoal = move_to_pose
        #     # sol = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreEndEffectorEnvCollisions)
        #     sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
        #
        # robot.SetActiveDOFValues(sol)

        pose_list = []
        # for offset in grasp_offsets:
        offset_matrix = np.identity(4)
        offset_matrix[0][3] = - gripper_offset
        offset_matrix[2][3] = offset_mean

        translated_pose = np.matmul(rotated_pose, offset_matrix)
        move_to_pose = np.matmul(translated_pose, wrist_pose_wrt_gripper)
        draw = misc.DrawAxes(env, move_to_pose)

        # with env:  # lock environment
        #     Tgoal = move_to_pose
        #     # sol = manip.FindIKSolution(Tgoal, IkFilterOptions.IgnoreEndEffectorEnvCollisions)
        #     sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions)
        # if sol is not None:
        #     robot.SetActiveDOFValues(sol)

        ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(move_to_pose, check_collisions=True)

        if len(ik_sols) > 0: 
            pose_list.append(ik_sols[0])

        # random.shuffle(pose_list)
        if len(pose_list) == 0:
            print("Failed Put Down Pose Generations")
        return pose_list
