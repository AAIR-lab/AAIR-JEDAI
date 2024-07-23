from openravepy import *
import Config
from src.Robots.Models import FetchOpenRaveRobotModel
from src.Robots.Models import UAVOpenRaveRobotModel
from src.Robots.Models import YuMiOpenRaveRobotModel
from src.MotionPlanners.OpenRaveMotionPlanner import *
import src.util as util
import copy
import numpy as np
from src.Parser.ActionConfigParserV2 import ActionConfigParserV2
import importlib

from openravepy.misc import *
from trac_ik_python.trac_ik import IK
import pdb
import time

class OpenRaveSimulator(object):
    environment = None
    robot_init_pose = None
    robots = None
    def __init__(self, env_xml=None, body_name_transform_map=None):
        RaveSetDebugLevel(DebugLevel.Error)
        if OpenRaveSimulator.environment is not None:
            self.env = OpenRaveSimulator.environment
            self.AVAILABLE_MOTION_PLANNERS = ['NATIVE','OMPL_RRTConnect']
            self.robot_init_pose = OpenRaveSimulator.robot_init_pose
            self.robots = OpenRaveSimulator.robots
            # if body_name_transform_map:
            #     self.__setup_environment(body_name_transform_map)

        else:
            self.env = Environment()
            OpenRaveSimulator.environment = self.env
            action_config_parser = ActionConfigParserV2(Config.LL_ACTION_CONFIG)
            self.robots = {}
            # OpenRaveSimulator.environment = None
            if env_xml:
                with self.env:
                    if Config.SHOW_VIEWER:
                        self.env.SetViewer(Config.VIEWER)
                    self.env.Load(env_xml)
                self.load_robot(action_config_parser.get_robots_list())
            elif body_name_transform_map:
                with self.env:
                    self.env.Load(Config.OPENRAVE_ENV_XML)
                    self.load_robot(action_config_parser.get_robots_list())
                    self.__setup_environment(body_name_transform_map)
            self.AVAILABLE_MOTION_PLANNERS = ['NATIVE']
            OpenRaveSimulator.robot_init_pose = self.robot_init_pose
            collision_checker = RaveCreateCollisionChecker(self.env, "pqp")
            collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
            self.env.SetCollisionChecker(collision_checker)
            OpenRaveSimulator.robots = self.robots
                # TODO create initial body_name_transform_map so that we can roll back

    def load_robot(self,robots):
        for robot in robots:
            gen_class = importlib.import_module('src.Robots.'+robots[robot])
            r = getattr(gen_class, robots[robot])(self.env, Config.REAL_ROBOT)
            self.robots[str(robot)] = r


    def get_set_all_body_names(self):
        return set(bdy.GetName() for bdy in self.env.GetBodies())

    # def show_viewer(self):
    #     self.env.SetViewer('qtcoin')

    def visualize_transform(self, T):
        DrawAxes(self.env, T)

    def get_transform_matrix(self, object_name):
        return self.env.GetKinBody(object_name).GetTransform()

    def set_transform_matrix(self, object_name, transform_matrix):
        return self.env.GetKinBody(object_name).SetTransform(transform_matrix)

    def get_object(self, object_name):
        return self.env.GetKinBody(object_name)

    def get_obj_name_box_extents(self, object_name):
        geom = self.env.GetKinBody(object_name).GetLink('base').GetGeometries()[0]
        o_x, o_y, o_z = geom.GetBoxExtents().tolist()
        return o_x, o_y, o_z

        return  object.GetBoxExtents()

    def get_object_link_dimensions(self, object_name, link_name):
        with self.env:
            obj = self.get_object(object_name)
            link = obj.GetLink(link_name)
            geom = link.GetGeometries()[0]
            height = geom.GetCylinderHeight()
            base_width = 0
            base_length = 0
        return base_width, base_length, height

    def get_object_link_geometries(self, object_name, link_name):
        obj = self.get_object(object_name)
        link = obj.GetLink(link_name)
        geom = link.GetGeometries()
        return geom


    def get_matrix_from_pose(self,w, rot_x, rot_y, rot_z, translation_x, translation_y, translation_z):
        return matrixFromPose((w, rot_x, rot_y, rot_z, translation_x, translation_y, translation_z))

    def get_matrix_from_axis_angle(self,rot_x, rot_y, rot_z):
        return matrixFromAxisAngle((rot_x, rot_y, rot_z))


    def __setup_environment(self, body_name_transform_map):

        available_bodies = self.get_set_all_body_names()

        for body_name in body_name_transform_map:
            if body_name not in available_bodies:
                raise OpenRaveSimulatorException(body_name + " is not in the loaded environment xml")
            self.env.GetKinBody(body_name).SetTransform(body_name_transform_map.get(body_name))

    def set_environment(self, values):
        with self.env:
            robots = self.env.GetRobots()
            for robot in robots:
                robot.ReleaseAllGrabbed()
            for body in self.env.GetBodies():
                if body.IsRobot():
                    continue
                body.SetTransform(values['objects'][body.GetName()]['transform'])

            for robot in robots:
                if robot is not None:
                    robot.ReleaseAllGrabbed()
                    robot.WaitForController(0)
                    # if robot.GetName() == 'yumi':
                    #     robot.SetTransform(values['robots'][robot.GetName()]['transform'])
                    # else:
                    #     robot.GetLink('base_link').SetTransform(values['robots'][robot.GetName()]['transform'])
                    robot.SetTransform(values['robots'][robot.GetName()]['transform'])
                    if 'dof_values' in values['robots'][robot.GetName()]:
                        robot.SetDOFValues(values['robots'][robot.GetName()]['dof_values'])
                    if 'active_arm' in values['robots'][robot.GetName()]:
                        robot.SetActiveManipulator(values['robots'][robot.GetName()]['active_arm'])
                    if "active_joint_indices" in values['robots'][robot.GetName()]:
                        if type(values['robots'][robot.GetName()]['active_joint_indices']) == type(-1):
                            self.robots[robot.GetName()].activate_base_joints()
                        else:
                            robot.SetActiveDOFs(values['robots'][robot.GetName()]['active_joint_indices'])
                    if 'grabbed_objects' in values['robots'][robot.GetName()]:
                        for obj_name in values['robots'][robot.GetName()]['grabbed_objects']:
                            robot.Grab(self.env.GetKinBody(obj_name))

        self.env.UpdatePublishedBodies()


    def get_env_and_robot(self, ll_state):
        #TODO Support multi robots
        env = self.env

        with env:
            robots = env.GetRobots()
            robot = None
            if len(robots) > 0:
                robot = robots[0]

            for body in env.GetBodies():
                if body.IsRobot():
                    continue
                body.SetTransform(ll_state.values['objects'][body.GetName()]['transform'])

            if robot is not None:
                if robot.GetName() == 'yumi':
                    robot.SetTransform(ll_state.values['robots'][robot.GetName()]['transform'])
                else:
                    robot.GetLink('base_link').SetTransform(ll_state.values['robots'][robot.GetName()]['transform'])
                if 'dof_values' in ll_state.values['robots'][robot.GetName()]:
                    robot.SetDOFValues(ll_state.values['robots'][robot.GetName()]['dof_values'])
                if 'grabbed_objects' in ll_state.values['robots'][robot.GetName()]:
                    for obj_name in ll_state.values['robots'][robot.GetName()]['grabbed_objects']:
                        robot.Grab(env.GetKinBody(obj_name))

        return env, robot

    def get_joint_values_from_waypoint(self, waypoint,robot):
        try:
            n = robot.GetActiveDOF()
        except Exception,e:
            print dir(robot)
            print e
            n = 8
        return waypoint[0:n]

    def set_robot_active_dof_values_to_waypoint_values(self, ll_state, waypoint):
        env, _ = self.get_env_and_robot(ll_state)
        with env:
            robot = env.GetRobots()[0]
            jv = self.get_joint_values_from_waypoint(waypoint,robot)
            robot.SetActiveDOFValues(jv)
        return env

    def get_body_name_associated_with_link(self, link):
        name = link.GetParent().GetName()
        return name

    def get_collision_report(self,env):
        cr = CollisionReport()
        with env:
            robot = env.GetRobots()[0]
            env.CheckCollision(robot, cr)
        return cr




    def get_colliding_objects(self, env):
        with env:
            collision_checker = RaveCreateCollisionChecker(env, "pqp")
            collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
            env.SetCollisionChecker(collision_checker)

            robot = env.GetRobots()[0]
            bodies = env.GetBodies()
            colliding_objects = []
            for body in bodies:
                if body.IsRobot():
                    continue
                elif env.CheckCollision(robot, body):
                    colliding_objects.append(body.GetName())
            return colliding_objects


    def remove_all_removable_bodies(self, env):

        action_config_parser = ActionConfigParserV2(Config.LL_ACTION_CONFIG)
        # print "Action Config Parser Generated"
        non_movable_bodies = action_config_parser.get_non_removable_bodies()
        # print "Got the list"

        grabbed_bodies = env.GetRobots()[0].GetGrabbed()
        # print "Got the grabbd object"
        if len(grabbed_bodies) > 0:
            grabbed_body_name = grabbed_bodies[0].GetName()
            non_movable_bodies.append(grabbed_body_name)

        # print "List Updated"
        name_to_object_and_transform = {}
        from multiprocessing import Lock
        lock = Lock()
        print env.GetBodies()
        # import IPython
        # IPython.embed()
        with env:
            for body in env.GetBodies():
                if not body.IsRobot() and body.GetName() not in non_movable_bodies:
                    body_name = body.GetName()
                    body_transform = body.GetTransform()
                    name_to_object_and_transform[body_name] = {'object': body, 'transform': body_transform}
                    new_body_transform = copy.deepcopy(body_transform)
                    new_body_transform[2,3] = 10
                    body.SetTransform(new_body_transform)
                    # self.env.Remove(body)

        print "Objects removed and transforms stored"

        return  name_to_object_and_transform


    def get_motion_plan(self, motion_planner, robot_name, body_name_transform_map,  list_active_joint_names, list_active_joint_indices,
                        goal_transform_values, ll_state, check_collision=True , dof = False,traj_type=None,current_pose = None):


        if motion_planner not in self.AVAILABLE_MOTION_PLANNERS:
            raise OpenRaveSimulatorException(motion_planner + "is not available in chosen simulator")

        # TODO Set active manipulator by dynamically constructing a manipulator from the given joint names or values
        # TODO __setup_environment should return openrave_env instead of relying on ll_state(OpenRaveLowLevelState), ll_state should be removed
        # self.__setup_environment(body_name_transform_map)


        # self.env , _ = self.get_env_and_robot(ll_state) #TODO this should be removed

        bodies_removed = False
        name_to_object_and_transform = {}
        robot = self.env.GetRobot(robot_name)

        if check_collision == False:
            name_to_object_and_transform = self.remove_all_removable_bodies(self.env)
            bodies_removed = True

        trajectory_object = None
        if type(goal_transform_values) == list or traj_type == "base":  #TODO, find another way to dermine if it is a base pose or use an motion planner interface that is common to all sorts of motion plan problems
            # goal_transform_values = goal_transform_values[0]
            # If it is a base pose
            # transform = goal_transform_values
            # with self.env:
            #     robot = self.env.GetRobot(robot_name)
            #     robot.SetTransform(goal_transform_values)
            with self.env:
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
                        trajectory_object,_,_ = self.rrt_connect_planner.get_mp_trajectory_to_goal(robot,goal_transform=g)
                        end_time = time.time()
                        print "time took:",end_time-start_time
                    robot.SetActiveDOFs(old_dof_values)
                except Exception,e:
                    print e
                    pass
        else:
            # with self.env:
            if not dof:
                iksolution = self.robots[robot_name].get_ik_solutions(goal_transform_values,check_collisions=True)
            else:
                iksolution = [goal_transform_values]
            i = 0
            while i < len(iksolution):
                selected_iksolution = iksolution[i]
                try:
                    if Config.MOTION_PLANNER == Config.OPENRAVE_NATIVE_MOTION_PLANNER:
                        trajectory_object = interfaces.BaseManipulation(robot).MoveActiveJoints(selected_iksolution,outputtrajobj=True, execute=True)
                    else:
                        with self.env:
                            try:
                                trajectory_object, is_success, fail_cause = self.rrt_connect_planner.get_mp_trajectory_to_goal(robot,goal_transform=selected_iksolution)
                            except Exception,e:
                                print e
                    break

                except Exception as ex:
                    trajectory_object = None
                    import traceback
                    print(traceback.format_exc())
                    return None
                    # exit(-1)

        # self.__reset_environment()

        with self.env:
            if bodies_removed:
                for body_name in  name_to_object_and_transform:
                    name_to_object_and_transform[body_name]['object'].SetTransform(name_to_object_and_transform[body_name]['transform'])


        return trajectory_object


class OpenRaveSimulatorException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message
