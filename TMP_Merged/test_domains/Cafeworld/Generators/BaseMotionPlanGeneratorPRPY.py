import copy
import numpy as np
import types

from openravepy import *
from openravepy.misc import *
from trac_ik_python.trac_ik import IK

import misc.OpenRaveHelper as OpenRaveHelper
from src.Simulators.OpenRaveSimulator import *  # OpenRaveSimulator, OpenRaveSimulatorException
import IPython
from src.DataStructures.Generator import Generator

from Config import ROBOT_NAME
from prpy.planning.base import PlanningError
from prpy.planning import sbpl
from prpy.planning.openrave import OpenRAVEPlanner
from openravepy import planningutils


class BaseMotionPlanGeneratorPRPY(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['pose_current', 'pose_end']
        super(BaseMotionPlanGeneratorPRPY, self).__init__(known_argument_values, required_values)
        '''

        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        :param goal_joint_values: list of joint values in the same order as that of list_active_joint_indices or list_active_joint_names

        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''
        self.ll_state = ll_state
        self.assume_refinable = known_argument_values.get("assume_refinable", False)
        # self.env = self.ll_state.simulator.env
        self.known_argument_values = known_argument_values
        self.robot_name = self.known_argument_values["robot"]
        self.robot = self.ll_state.simulator.env.GetRobot(self.robot_name)
        self.current_pose = self.known_argument_values.get('pose_current')
        self.generated_val = self.known_argument_values.get('pose_end')
        self.goal_config_generator()
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        self.planner = OpenRAVEPlanner()
        # self.planner.setupEnv(self.ll_state.simulator.env)
        # This is from herbpy, but happens to work for fetch as well
        # TODO: Generate yaml file for fetch

    def goal_config_generator(self):
        # If goal from generator is a joint configuration list, return list. Otherwise, generate IKSolution
        generated_val_shape = np.asarray(self.generated_val).shape
        assert generated_val_shape == (3,), "Only 3x1  list of base_x, base_y, base_z or base_rot are acceptable goals"
        self.current_pose = self.robot.GetTransform()
        translation = np.eye(4)
        translation[0:1, 3] = self.generated_val[:1]
        rotation_about_z = matrixFromAxisAngle(np.asarray([0., 0., self.generated_val[2]]))
        self.goal_base_pose = np.matmul(translation, rotation_about_z)

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

    def get_traj_from_planner(self, base_goal_val):
        try:
            trajectory_object = self.planner.PlanToConfiguration(self.robot, self.generated_val)
        except PlanningError:
            print(__file__.split('/')[-1], ": Could not find base motion plan")
            return None
        status = planningutils.SmoothAffineTrajectory(trajectory_object, [4., 4., 4.], [.5, .5, .5],
                                                      plannername='LinearTrajectoryRetimer')
        if status == PlannerStatus.HasSolution:
            trajectory_object = trajectory_object.serialize()
        else:
            print(__file__.split('/')[-1], "Planning Failed: Failed to smooth affine trajectory")
            trajectory_object = None
        return trajectory_object

    def get_motion_plan(self, remove_bodies=True):
        # Delete bodies if remove_bodies is true (Err mode)
        old_active_dofs = self.robot.GetActiveDOFIndices()
        self.robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
        if remove_bodies:
            name_to_object_and_transform = {}
            name_to_object_and_transform = self.ll_state.simulator.remove_all_removable_bodies(
                self.ll_state.simulator.env)

        trajectory_object = None
        with self.ll_state.simulator.env:
            if abs(np.sum(self.current_pose - np.array(self.goal_base_pose))) < 1e-3:
                trajectory_object = True
            elif self.assume_refinable:
                
                
                goal_config = self.generated_val
                
                trajectory_object = RaveCreateTrajectory(self.ll_state.simulator.env, "")
                cspec = self.robot.GetActiveConfigurationSpecification("linear")
                trajectory_object.Init(cspec)
                trajectory_object.Insert(0, goal_config)
                
                _ = planningutils.RetimeTrajectory(trajectory_object)
                trajectory_object = trajectory_object.serialize()
            else:
                # self.robot.SetActiveDOFValues(self.current_pose)
                # trajectory_object = 1
                trajectory_object = self.get_traj_from_planner(self.goal_base_pose)
                
        # Add bodies back if deleted previously
        if remove_bodies:
            with self.ll_state.simulator.env:
                for body_name in name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(
                        name_to_object_and_transform[body_name]['transform'])
        self.robot.SetActiveDOFs(old_active_dofs)
        return trajectory_object

    def generate_function_err(self):
        for mp_plan_no in range(3):
            trajectory_object = self.get_motion_plan(remove_bodies=True)
            if trajectory_object is None:
                raise StopIteration
            yield trajectory_object
        raise StopIteration

    def generate_function_err_free(self):
        for mp_plan_no in range(1):
            for attempts in range(1):
                trajectory_object = self.get_motion_plan(remove_bodies=False)
                if trajectory_object is not None:
                    break
            if trajectory_object is None:
                raise StopIteration
            yield trajectory_object
        raise StopIteration

    def get_next(self, extra):
        if extra["flag"]:
            return self.generate_function_state_err_free.next()
        else:
            return self.generate_function_state_err.next()
