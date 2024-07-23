import copy
import numpy as np
import types

from openravepy import *
from openravepy.misc import *

import misc.OpenRaveHelper as OpenRaveHelper
from src.Simulators.OpenRaveSimulator import * #OpenRaveSimulator, OpenRaveSimulatorException
import IPython
from src.DataStructures.Generator import Generator

# from Config import IK_SOLVER, 
ROBOT_MANIP_DOF=7
from prpy.planning.base import PlanningError
from prpy.planning import ompl, CBiRRTPlanner, chomp
from openravepy import planningutils

class MotionPlanGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        required_values = ['pose_current', 'pose_end','robot']
        super(MotionPlanGenerator, self).__init__(known_argument_values, required_values)
        '''
        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''
        self.ll_state = ll_state
        self.known_argument_values = known_argument_values
        self.robot_name = known_argument_values.get('robot')
        self.robot = self.ll_state.simulator.env.GetRobot(self.robot_name)
        self.current_pose = known_argument_values.get('pose_current')
        self.generated_val = known_argument_values.get('pose_end')
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        # self.planner = ompl.OMPLPlanner('RRTConnect')
        self.planner = CBiRRTPlanner(timelimit=20.00)
        self.simplifier = ompl.OMPLSimplifier()
    
    
    def goal_param_generator(self):
        # Generate type of goal based on pose_end
        generated_val_shape = np.asarray(self.generated_val).shape
        assert generated_val_shape == (4,4) or generated_val_shape == (ROBOT_MANIP_DOF,), "Only 4x4 pose matrix or list of joint " \
            "state values of size ROBOT_MANIP_DOF = "+str(ROBOT_MANIP_DOF)+" are acceptable goal values"
        # If goal from generator is a joint configuration list, return list.
        if generated_val_shape == (ROBOT_MANIP_DOF,):
            self.goal_configs = [self.generated_val]
        # If goal from generator is a pose, find IK solutions for pose.
        elif generated_val_shape == (4,4):
            self.goal_pose = self.generated_val
            self.goal_configs = self.ll_state.simulator.robots[self.known_argument_values["robot"]].get_ik_solutions(self.goal_pose, check_collisions=True)

        else:
            # TODO support for TSR
            print(__file__.split('/')[-1],'Goal is neither configuration list nor pose') # This will never occur

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
    
    def planToConfigurations(self,goal_configs):
        # Motion Planning to reach joint state value(s)
        # Get trajectory from planner based on type of goal config passed
        # ( config a.k.a ik solutions a.k.a joint states )
        try:
            if len(goal_configs) == 1:
                # If goal is a single IK solution, then call PlanToConfiguration
                trajectory_object = self.planner.PlanToConfiguration(self.robot, goal_configs[0],timelimit=20.00)
            else:
                # If goal is a list of IK solutions, then call PlanToConfigurations
                trajectory_object = self.planner.PlanToConfigurations(self.robot, goal_configs,timelimit=20.00)
            trajectory_object = self.simplifier.ShortcutPath(self.robot,trajectory_object)
        except:
            print(__file__.split('/')[-1],": Could not find motion plan")
            return None
        # Retime and serialize the trajectory
        _ = planningutils.RetimeTrajectory(trajectory_object)
        trajectory_object = trajectory_object.serialize()
        return trajectory_object


    def get_motion_plan(self,remove_bodies=True):
        # Generate trajectory based on planner capability
        
        trajectory_object = None
        # Delete bodies if remove_bodies is true (Err mode)
        if remove_bodies:
            name_to_object_and_transform = {}
            name_to_object_and_transform = self.ll_state.simulator.remove_all_removable_bodies(self.ll_state.simulator.env)
        
        self.goal_param_generator()

        # If goal is a single / list of configurations
        if hasattr(self, 'goal_configs'):
            if self.planner.has_planning_method('PlanToConfigurations'):
                # Check if plan exists for any of the goal configs in one shot
                i = 0
                while i < 5:
                    self.goal_param_generator()
                    with self.ll_state.simulator.env:
                        trajectory_object = self.planToConfigurations(self.goal_configs)
                    if trajectory_object is not None:
                        break
                    i += 1
            
            elif self.planner.has_planning_method('PlanToConfiguration'):
                # Check if plan exists for each goal config at a time
                i = 0
                while i < len(self.goal_configs):
                    selected_goal_config =  self.goal_configs[i]
                    with self.ll_state.simulator.env:
                        trajectory_object = self.planToConfigurations([selected_goal_config])
                    i+=1
                    if trajectory_object is not None:
                        break
        else:
            # TODO add TSR / TSR chain support
            print("Cannot plan with configuration")
        
        # Add bodies back if deleted previously
        if remove_bodies:
            with self.ll_state.simulator.env:
                for body_name in  name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])
        return trajectory_object
        
    def generate_function_err(self):
        for mp_plan_no in range(3):
            trajectory_object = self.get_motion_plan(remove_bodies = True)
            if trajectory_object is None:
                raise StopIteration
            yield trajectory_object
        raise StopIteration


    def generate_function_err_free(self):
        for mp_plan_no in range(10):
            for attempts in range(3):
                trajectory_object = self.get_motion_plan( remove_bodies = False)
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