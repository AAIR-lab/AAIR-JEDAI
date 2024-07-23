import random
from src.DataStructures.Generator import Generator
import src.OpenraveUtils as OpenraveUtils
import numpy as np
import openravepy as orpy
import math
import copy
import Config



class PutDownPoseGeneratorKeva(Generator):
    if Config.LOOPED_RUNS:
        run_num = Config.get_run_number()
        if run_num > 1:
            root_plank = 'plank1'
        else:
            root_plank = ''
    else:
        root_plank = ''

    def __init__(self, ll_state=None, known_argument_values=None):

        required_values = ['robot', 'obj']

        super(PutDownPoseGeneratorKeva, self).__init__(known_argument_values, required_values)
        self.simulator = ll_state.simulator
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()
        self.table_name = 'work_table_center'
        self.plank_name = known_argument_values.get('obj')
        if Config.LOOPED_RUNS:
            self.plank_name = Config.correct_plank_name(self.plank_name)
        #self.putdown_side = known_argument_values.get('region')
        self.reference_structure_path = Config.REFERENCE_STRUCTURE_PATH
        self.robot = self.simulator.env.GetRobots()[0]


    def reset(self):
        if self.plank_name == PutDownPoseGeneratorKeva.root_plank:
            PutDownPoseGeneratorKeva.root_plank = ""
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        for gt in self._compute_put_down_pose_list():
            yield gt

    def get_next(self,flag):
        return self.generate_function_state.next()

    def _compute_put_down_pose_list(self):
        # Tref - reference env
        # T - current environment
        env = self.simulator.env
        world_T_robot = self.robot.GetTransform()
        robot_T_world = np.linalg.inv(world_T_robot)

        # Code from grasp pose generator
        # plank_T_gripper = np.eye(4)
        # plank_T_gripper[1,3]= -0.135 #gripper_offset
        # t1 = matrixFromAxisAngle([ 0, -np.pi/2, 0])
        # t2 = matrixFromAxisAngle([-np.pi/2, 0, 0])
        # plank_T_gripper = np.matmul(np.matmul(plank_T_gripper,t1),t2)

        # Actual pose from env
        world_T_gripper = self.robot.GetActiveManipulator().GetEndEffectorTransform()
        world_T_curr_plank = env.GetKinBody(self.plank_name).GetTransform()
        plank_T_gripper = np.matmul(np.linalg.inv(world_T_curr_plank),world_T_gripper)

        reference_env = orpy.Environment()
        reference_env.Load(self.reference_structure_path)


        table = self.simulator.env.GetKinBody('table60')

        if table is None:
            table = self.simulator.env.GetKinBody("work_table_center")
        world_T_table_surface = table.GetTransform()

        if self.plank_name == PutDownPoseGeneratorKeva.root_plank:
            PutDownPoseGeneratorKeva.root_plank = ""

        put_down_pose_list = []
        if len(PutDownPoseGeneratorKeva.root_plank) == 0:
            number_of_poses = 10
            putdown_x = []
            putdown_y = []
            for i in range(number_of_poses):
                putdown_x.append(random.uniform(0.5, 0.55))
                putdown_y.append(random.uniform(-0.12, 0.05)) #For PI envs

            ref_root_plank = reference_env.GetKinBody(self.plank_name)
            world_Tref_root_plank = ref_root_plank.GetTransform()
            z = ref_root_plank.ComputeAABB().extents()[2]
            z += world_T_table_surface[2,3]+0.1

            for x, y in zip(putdown_x, putdown_y):
                world_T_root_plank = world_Tref_root_plank.copy()
                world_T_root_plank[:3,3] = [x,y,z]

                robot_T_gripper = np.matmul(np.matmul(robot_T_world,world_T_root_plank),plank_T_gripper)
                # openrave requires gripper in world frame
                world_T_gripper = np.matmul(world_T_robot,robot_T_gripper)
                draw = orpy.misc.DrawAxes(env, world_T_gripper)
                ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(world_T_gripper, check_collisions=True)
                if len(ik_sols)>0:
                    put_down_pose_list.append(world_T_gripper)
            PutDownPoseGeneratorKeva.root_plank = self.plank_name

        else:
            #From reference environment
            world_Tref_root_plank = reference_env.GetKinBody(PutDownPoseGeneratorKeva.root_plank).GetTransform()
            root_plank_Tref_world = np.linalg.inv(world_Tref_root_plank)
            world_Tref_curr_plank = reference_env.GetKinBody(self.plank_name).GetTransform()
            root_plank_T_curr_plank = np.matmul(root_plank_Tref_world,world_Tref_curr_plank)

            # Curr env
            world_T_root_plank = env.GetKinBody(PutDownPoseGeneratorKeva.root_plank).GetTransform()
            world_T_curr_plank = np.matmul(world_T_root_plank,root_plank_T_curr_plank)
            robot_T_gripper = np.matmul(np.matmul(robot_T_world,world_T_curr_plank),plank_T_gripper)

            # openrave requires gripper in world frame
            world_T_gripper = np.matmul(world_T_robot,robot_T_gripper)
            z = reference_env.GetKinBody(self.plank_name).ComputeAABB().extents()[2]
            # if z < 0.004:
            #     world_T_gripper[2,3] += 0.01
            draw = orpy.misc.DrawAxes(env, world_T_gripper)

            ik_sols = self.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(world_T_gripper, check_collisions=True)
            if len(ik_sols)>0:
                put_down_pose_list.append(world_T_gripper)

        if len(put_down_pose_list) == 0:
            print "No valid solution found"
        return put_down_pose_list
