import copy
import misc.OpenRaveHelper as OpenRaveHelper
from src.MotionPlanners import OpenRaveMotionPlanner
from src.Simulators.OpenRaveSimulator import *
from src.DataStructures.Generator import Generator
from openravepy import *
import Config
from src.MotionPlanners.OpenRaveMotionPlanner import *


class MotionPlanGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        # required_values = ['pose_current', 'pose_end', 'robot_name', 'goal_joint_values', 'list_active_joint_names', 'list_active_joint_indices']

        required_values = ['pose_current', 'pose_end']
        super(MotionPlanGenerator, self).__init__(known_argument_values, required_values)

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
        self.simulator = ll_state.simulator  # Config.SIMULATOR
        self.motion_planner = 'NATIVE'  # Config.MOTION_PLANNER
        self.robot_name = known_argument_values["robot"]
        self.robot = self.simulator.env.GetRobot(self.robot_name)
        self.list_active_joint_names = ['Unknown']
        self.list_active_joint_indices = ['Unknown']
        self.current_pose = known_argument_values.get('pose_current')
        self.goal_joint_values = known_argument_values.get('pose_end')
        self.ll_state = ll_state
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()
        self.number_of_values = 10
        self.known_argument_values = known_argument_values
        self.planner = OpenRave_OMPL_Base_Planner(self.simulator.env, self.robot, planner=Config.MOTION_PLANNER)

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()
        self.generate_function_state_err = self.generate_function_err()

    def get_motion_plan(self, goal_transform_values, check_collision=True, dof=False):

        bodies_removed = False
        name_to_object_and_transform = {}
        robot = self.simulator.env.GetRobot(self.known_argument_values["robot"])

        if check_collision == False:
            name_to_object_and_transform = self.simulator.remove_all_removable_bodies(self.simulator.env)
            bodies_removed = True

        trajectory_object = None
        # with self.env:
        if not dof:
            iksolution = self.simulator.robots[self.robot_name].get_ik_solutions(goal_transform_values,check_collisions=True)
        else:
            iksolution = [goal_transform_values]
        i = 0
        while i < len(iksolution) and i < Config.MAX_IKs_TO_CHECK_FOR_MP:
            selected_iksolution = iksolution[i]
            with self.simulator.env:
                try:
                    trajectory_object, is_success, fail_cause = self.planner.get_mp_trajectory_to_goal(
                        robot, goal_transform=selected_iksolution)
                except Exception, e:
                    print e
                else:
                    break
                    # exit(-1)

        # self.__reset_environment()

        with self.simulator.env:
            if bodies_removed:
                for body_name in name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(
                        name_to_object_and_transform[body_name]['transform'])

        return trajectory_object

    def generate_function_err(self):
        for mp_plan_no in range(1):
            end_dof = False
            if len(list(self.goal_joint_values)) == 8:
                end_dof = True
            mp = self.get_motion_plan(self.goal_joint_values, check_collision=False, dof=end_dof)
            if mp is None:
                raise StopIteration
            yield mp
        raise StopIteration

    def generate_function_err_free(self):
        for mp_plan_no in range(3):
            end_dof = False
            mp = None
            if len(list(self.goal_joint_values)) == 8:
                end_dof = True
            for attempts in range(5):
                mp = self.get_motion_plan(self.goal_joint_values, check_collision=True, dof=end_dof)
                if mp is not None:
                    break
            if mp is None:
                raise StopIteration
            yield mp
        raise StopIteration

    def get_next(self, extra):
        if extra["flag"]:
            return self.generate_function_state_err_free.next()
        else:
            return self.generate_function_state_err.next()

