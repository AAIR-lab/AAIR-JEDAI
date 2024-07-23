import copy
import misc.OpenRaveHelper as OpenRaveHelper
from src.MotionPlanners import OpenRaveMotionPlanner
from src.Simulators.OpenRaveSimulator import *
import numpy as np
import openravepy

from src.DataStructures.Generator import Generator

import Config


class TrajectoryGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        # required_values = ['pose_current', 'pose_end', 'robot_name', 'goal_joint_values', 'list_active_joint_names', 'list_active_joint_indices']

        required_values = ['pose_current', 'pose_end']
        super(TrajectoryGenerator, self).__init__(known_argument_values, required_values)

        '''

        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        :param list_active_joint_names:
        :param list_active_joint_indices:
        :param goal_joint_values: list of joint values in the same order as that of list_active_joint_indices or list_active_joint_names

        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''
        self.object_name_to_transform_map = {'Unknown': 'Unknown'}
        self.simulator = OpenRaveSimulator(Config.OPENRAVE_ENV_XML)  # Config.SIMULATOR
        self.motion_planner = Config.MOTION_PLANNER
        self.robot_name = Config.ROBOT_NAME
        self.list_active_joint_names = ['Unknown']
        self.list_active_joint_indices = ['Unknown']
        self.goal_pose = np.array(known_argument_values.get('pose_end'))
        self.ll_state = ll_state
        self.known_arguments = known_argument_values
        self.move_trajectory_generator_state = self.move_trajectory_generator()
        # self.inspect_trajectory_generator_state = self.inspect_trajectory_generator()

    def reset(self):
        self.move_trajectory_generator_state = self.move_trajectory_generator()
        # self.inspect_trajectory_generator_state = self.inspect_trajectory_generator()

    def move_trajectory_generator(self):
        i = 0
        while True:
            if i > 5:
                raise StopIteration
            env = self.simulator.env
            # import IPython
            # IPython.embed()
            robot = env.GetRobot(self.robot_name)
            basemanip = openravepy.interfaces.BaseManipulation(robot)
            mp = basemanip.MoveActiveJoints(goal=self.goal_pose,execute=False,outputtraj=True)
            # # mp = self.simulator.get_motion_plan(self.motion_planner, self.robot_name, self.object_name_to_transform_map,
            #                                     self.list_active_joint_names, self.list_active_joint_indices,
            #                                     self.goal_pose,
            #                                     self.ll_state, dof=True , traj_type="base")
            i += 1
            yield mp

    def inspect_trajectory_generator(self):
        i = 0
        while True:
            if i > 5:
                raise StopIteration
            env = self.simulator.env
            robot = env.GetRobot(self.robot_name)
            basemanip = openravepy.interfaces.BaseManipulation(robot)
            init_transform = robot.GetLink('base_link').GetTransform()
            trajs = []
            for i in range(self.goal_pose.shape[0]):
                trajs.append(basemanip.MoveActiveJoints(goal = self.goal_pose[i,:],execute = False, outputtraj = True))
                robot.SetActiveDOFValues(self.goal_pose[i,:])
            robot.GetLink('base_link').SetTransform(init_transform)
            combined_traj = openravepy.RaveCreateTrajectory(env,'')
            trajobj = openravepy.RaveCreateTrajectory(env,'')
            openravepy.Trajectory.deserialize(trajobj,trajs[0])
            combined_traj.Init(trajobj.GetConfigurationSpecification())
            k = 0
            for traj in trajs:
                trajobj = openravepy.RaveCreateTrajectory(env, '')
                try:
                    openravepy.Trajectory.deserialize(trajobj, traj)
                    for j in range(trajobj.GetNumWaypoints()):
                        print trajobj.GetWaypoint(j)
                        try:
                            combined_traj.Insert(k,trajobj.GetWaypoint(j))
                        except:
                            pass
                        else:
                            k+=1
                except:
                    pass
            temp = combined_traj.serialize()
            i += 1
            yield combined_traj.serialize()


    def get_next(self, extra):
        traj_type = extra["type"]
        if traj_type == "BaseTrajectory":
            return self.move_trajectory_generator_state.next()
        elif traj_type == "InspectTrajectory":
            return self.inspect_trajectory_generator().next()

