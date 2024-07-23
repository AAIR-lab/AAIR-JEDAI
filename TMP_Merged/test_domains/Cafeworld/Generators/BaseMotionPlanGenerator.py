import copy
import misc.OpenRaveHelper as OpenRaveHelper
from src.MotionPlanners import OpenRaveMotionPlanner
from src.Simulators.OpenRaveSimulator import *
from src.DataStructures.Generator import Generator
from openravepy import *
import Config
from src.MotionPlanners.OpenRaveMotionPlanner import *


class BaseMotionPlanGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        # required_values = ['pose_current', 'pose_end', 'robot_name', 'goal_joint_values', 'list_active_joint_names', 'list_active_joint_indices']

        required_values = ['pose_current', 'pose_end']
        super(BaseMotionPlanGenerator, self).__init__(known_argument_values, required_values)

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

    def get_motion_plan(self, goal_transform_values, check_collision=True, current_pose = None):

        bodies_removed = False
        name_to_object_and_transform = {}
        robot = self.simulator.env.GetRobot(self.known_argument_values["robot"])

        if check_collision == False:
            name_to_object_and_transform = self.simulator.remove_all_removable_bodies(self.simulator.env)
            bodies_removed = True

        trajectory_object = None

        with self.simulator.env:
            try:
                # robot.SetTransform(goal_transform_values)
                # trajectory_object = True
                # basemanip = interfaces.BaseManipulation(robot)

                g = goal_transform_values
                old_dof_values = robot.GetActiveDOFIndices()
                robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in Config.ROBOT_BASE_JOINTS])
                current_dof_vals = robot.GetActiveDOFValues()
                if abs(np.sum(current_dof_vals - np.array(g))) < 1e-3:
                    trajectory_object = True
                else:
                    import time
                    start_time = time.time()
                    robot.SetActiveDOFValues(current_pose)
                    trajectory_object, _, _ = self.planner.get_mp_trajectory_to_goal(robot, goal_transform=g)
                    end_time = time.time()
                    print "time took:", end_time - start_time
                robot.SetActiveDOFs(old_dof_values)
            except Exception, e:
                print e
                pass

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
            mp = self.get_motion_plan(self.goal_joint_values, check_collision=False, current_pose=self.current_pose)
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
                mp = self.get_motion_plan(self.goal_joint_values, check_collision=True, current_pose= self.current_pose)
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

