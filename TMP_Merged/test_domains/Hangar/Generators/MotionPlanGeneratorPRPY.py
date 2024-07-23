import copy
import numpy as np
import types

from openravepy import *
from openravepy.misc import *
from trac_ik_python.trac_ik import IK

import misc.OpenRaveHelper as OpenRaveHelper
from src.Simulators.OpenRaveSimulator import * #OpenRaveSimulator, OpenRaveSimulatorException
import IPython
from src.DataStructures.Generator import Generator

from Config import ROBOT_NAME, MAX_IKs_TO_CHECK_FOR_MP, IK_SOLVER, MOTION_PLANNER
from prpy.planning import ompl, CBiRRTPlanner
from openravepy import planningutils


def general_planner(robot,goal_joint_val):
    # planner = ompl.OMPLPlanner('RRTConnect')
    planner = ompl.OMPLPlanner(MOTION_PLANNER)
    simplifier = ompl.OMPLSimplifier()
    try:
        trajectory_object = planner.PlanToConfiguration(robot, goal_joint_val)
    except Exception,e:
        print e
        return None
    trajectory_object = simplifier.ShortcutPath(robot,trajectory_object)
    return trajectory_object

    
def cbirrt_delta_planner(robot,direction, distance):
    planner = CBiRRTPlanner(timelimit=10.)
    try:
        trajectory_object = planner.PlanToEndEffectorOffset(robot, direction, distance,smoothingitrs=2)
    except Exception,e:
        print e
        return None
    return trajectory_object


class MotionPlanGeneratorPRPY(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['pose_current', 'pose_end']
        super(MotionPlanGeneratorPRPY, self).__init__(known_argument_values, required_values)

        '''

        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        :param goal_joint_values: list of joint values in the same order as that of list_active_joint_indices or list_active_joint_names

        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''
        self.ll_state = ll_state
        # self.env = self.ll_state.simulator.env
        self.robot_name = ROBOT_NAME
        self.robot = self.ll_state.simulator.env.GetRobot(self.robot_name)
        self.current_pose = known_argument_values.get('pose_current')
        self.generated_val = known_argument_values.get('pose_end')

        generated_val_shape = np.asarray(self.generated_val).shape

        if generated_val_shape == (4,4) or generated_val_shape == (8,):
            self.traj_function = general_planner
            self.type = 'general'
            if generated_val_shape == (8,): #dof
                self.goal_joint_values = [self.generated_val]
            else: # Pose
                # IPython.embed()
                self.goal_pose = self.generated_val
                if IK_SOLVER == 'ik_fast':
                    self.goal_joint_values = self.ll_state.simulator.get_ik_solutions(self.robot, self.goal_pose, check_collisions=True)
                elif IK_SOLVER == 'trac_ik':
                    self.goal_joint_values = self.ll_state.simulator.get_trac_ik_solutions(self.robot, self.goal_pose, check_collisions=True)
                if len(self.goal_joint_values) > 1:
                    print(self.goal_joint_values)
                print("####################################"+str(len(self.goal_joint_values))+"###########################################")
            
        elif generated_val_shape == (4,): #distance and delta
            self.traj_function = cbirrt_delta_planner
            self.direction = self.generated_val[:3]
            self.distance = self.generated_val[3]
            self.type = 'delta_path'
        
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
    
    def planner_selector(self,col_check=True):
        if col_check:
            name_to_object_and_transform = {}
            name_to_object_and_transform = self.ll_state.simulator.remove_all_removable_bodies(self.ll_state.simulator.env)
        trajectory_object = None
        if self.type == 'general':
            i = 0
            while i < len(self.goal_joint_values):
                selected_iksolution =  self.goal_joint_values[i]
                # try:
                with self.ll_state.simulator.env:
                    trajectory_object = self.traj_function(self.robot,selected_iksolution)
                i+=1
                if trajectory_object is not None:
                    break
                # except Exception as ex:
                #     import traceback
                #     print(traceback.format_exc())
                #     print("################Should not be here####################")
                #     return None
        elif self.type == 'delta_path':
            # try:
            trajectory_object = self.traj_function(self.robot,direction, distance)
            # except Exception,e:
            #     print e
        if trajectory_object is not None:
            res = planningutils.RetimeTrajectory(trajectory_object)
            trajectory_object = trajectory_object.serialize()
        else:
            print("Noooooo")
        if col_check:
            with self.ll_state.simulator.env:
                for body_name in  name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])
        return trajectory_object
        
    def generate_function_err(self):
        for mp_plan_no in range(3):
            trajectory_object = self.planner_selector(col_check = False)
            if trajectory_object is None:
                raise StopIteration
            yield trajectory_object
        raise StopIteration


    def generate_function_err_free(self):
        for mp_plan_no in range(3):
            for attempts in range(5):
                trajectory_object = self.planner_selector( col_check = True)
                if trajectory_object is not None:
                    break;
            if trajectory_object is None:
                raise StopIteration
            yield trajectory_object
        raise StopIteration

    def get_next(self,extra):
        if extra["flag"]:
            return self.generate_function_state_err_free.next()
        else:
            return self.generate_function_state_err.next()