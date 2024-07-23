import copy
import numpy as np
import types

from openravepy import *
from openravepy.misc import *

import misc.OpenRaveHelper as OpenRaveHelper
from src.Simulators.OpenRaveSimulator import * #OpenRaveSimulator, OpenRaveSimulatorException
import IPython
from src.DataStructures.Generator import Generator

from prpy.planning.base import PlanningError
from prpy.planning import CBiRRTPlanner, mk
from openravepy import planningutils

import random

class DeltaMotionPlanGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['pose_current', 'pose_end']
        super(DeltaMotionPlanGenerator, self).__init__(known_argument_values, required_values)

        '''

        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        :param goal_joint_values: list of joint values in the same order as that of list_active_joint_indices or list_active_joint_names

        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''
        self.ll_state = ll_state
        self.assume_refinable = known_argument_values.get("assume_refinable", False)
        self.robot_name = known_argument_values.get('robot')
        self.robot = self.ll_state.simulator.env.GetRobot(self.robot_name)
        self.current_pose = known_argument_values.get('pose_current')
        self.generated_val = known_argument_values.get('pose_end')
        self.generate_function_state_err_free = self.generate_function_err_free()
        # self.generate_function_state_err = self.generate_function_err()
        self.distance_direction_generation()
        self.planner = CBiRRTPlanner(timelimit=20.00)
        # self.planner = mk.MKPlanner()

    def distance_direction_generation(self):
        # If goal from generator is a joint configuration list, return list. Otherwise, generate IKSolution
        generated_val_shape = np.asarray(self.generated_val).shape
        current_pose_shape = np.asarray(self.current_pose).shape
        assert generated_val_shape == (4,4) and current_pose_shape == (4,4), "current and goal values must be 4x4 pose matrix for " \
            +"DeltaMotionPlanGeneratorPRPY"
        difference_mat = self.generated_val - self.current_pose
        assert np.sum(difference_mat[:,:3])+difference_mat[3,3] == 0, "No rotation between current and goal pose allowed for delta traj calculations"
        self.v_3D = difference_mat[:3,3]
        self.distance = np.sqrt(np.sum(self.v_3D**2))
        self.direction = list(self.v_3D/self.distance)

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        # self.generate_function_state_err = self.generate_function_err()

        
    def get_traj_from_planner(self):
        try:
            trajectory_object = self.planner.PlanToEndEffectorOffset(self.robot, self.direction, self.distance,smoothingitrs=20,timelimit=20.)
            # trajectory_object = self.planner.PlanToEndEffectorOffset(self.robot, self.direction, self.distance)
        except PlanningError as e:
            print(__file__.split('/')[-1],": Could not find delta motion plan",e)
            return None
        _ = planningutils.RetimeTrajectory(trajectory_object)
        trajectory_object = trajectory_object.serialize()
        return trajectory_object

    def get_motion_plan(self):
        # Delete bodies as delta trajectories assume no collisions
        # with self.ll_state.simulator.env:
        #     name_to_object_and_transform = {}
        #     name_to_object_and_transform = self.ll_state.simulator.remove_all_removable_bodies(self.ll_state.simulator.env)
        
        if self.assume_refinable:
            
            ik_sols = self.ll_state.simulator.robots[self.robot_name].get_ik_solutions(self.generated_val, check_collisions=True)
            if len(ik_sols) == 0:
                return None
            ik_sol = random.choice(ik_sols)
            trajectory_object = RaveCreateTrajectory(self.ll_state.simulator.env, "")
            cspec = self.robot.GetActiveConfigurationSpecification("linear")
            trajectory_object.Init(cspec)
            trajectory_object.Insert(0, ik_sol)
            _ = planningutils.RetimeTrajectory(trajectory_object)
            trajectory_object = trajectory_object.serialize()
        else:
            
            trajectory_object = self.get_traj_from_planner()

        # Add bodies back if deleted previously
        # with self.ll_state.simulator.env:
        #     for body_name in  name_to_object_and_transform:
        #         name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])
        return trajectory_object


    def generate_function_err_free(self):
        for mp_plan_no in range(5):
            for attempts in range(1):
                trajectory_object = self.get_motion_plan()
                if trajectory_object is not None:
                    break;
            if trajectory_object is None:
                raise StopIteration
            yield trajectory_object
        raise StopIteration

    def get_next(self,extra):
        return self.generate_function_state_err_free.next()