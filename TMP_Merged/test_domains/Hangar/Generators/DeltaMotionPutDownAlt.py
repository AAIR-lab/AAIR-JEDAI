import copy
import misc.OpenRaveHelper as OpenRaveHelper
from src.MotionPlanners import OpenRaveMotionPlanner
from src.Simulators.OpenRaveSimulator import *
import openravepy
import src.OpenraveUtils as OpenraveUtils
import numpy as np
from src.DataStructures.Generator import Generator
import Config


class DeltaMotionPutDownAlt(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        # required_values = ['pose_current', 'pose_end', 'robot_name', 'goal_pose', 'list_active_joint_names', 'list_active_joint_indices']

        required_values = ['pose_current', 'pose_end']
        super(DeltaMotionPutDownAlt, self).__init__(known_argument_values, required_values)

        '''

        :param ll_state: is a mapping between object_name and transform matrix
        :param robot_name: string
        :param list_active_joint_names:
        :param list_active_joint_indices:
        :param goal_pose: list of joint values in the same order as that of list_active_joint_indices or list_active_joint_names

        '''
        '''low_level_environment is a mapping between object names and their transforms
            this can be used to construct a working environment model in most simulators'''

        self.object_name_to_transform_map = {'Unknown': 'Unknown'}
        self.simulator = OpenRaveSimulator(Config.OPENRAVE_ENV_XML)  # Config.SIMULATOR
        self.motion_planner = Config.MOTION_PLANNER
        self.robot_name = Config.ROBOT_NAME
        self.robot = self.simulator.env.GetRobot(self.robot_name)
        self.list_active_joint_names = ['Unknown']
        self.list_active_joint_indices = ['Unknown']
        self.current_pose = np.array(known_argument_values.get('pose_current'))
        self.goal_pose = np.array(known_argument_values.get('pose_end'))
        self.offset = self.set_offset()
        print("pregrasp offset = ", self.offset)
        self.number_of_values = 5
        self.step_size = self.offset / self.number_of_values
        self.ll_state = ll_state
        self.generate_function_state_err_free = self.generate_function_err_free()

    def set_offset(self):
        gf = self.robot.GetActiveManipulator().GetEndEffector().GetTransform()
        a = self.current_pose.dot(gf)
        b = self.goal_pose.dot(gf)
        return (b - a)[2, 3]

    def reset(self):
        self.generate_function_state_err_free = self.generate_function_err_free()

    def set_IK_solver(self, base, tip, urdf_str):
        ik_solver = IK(base, tip, urdf_string=urdf_str)
        return ik_solver

    def get_IK_solution(self, ik_solver, seed_state, trans, quat):
        max_count = 100
        count = 0
        while True and count < max_count:
            sol = ik_solver.get_ik(seed_state,
                                   trans[0], trans[1], trans[2],  # X, Y, Z
                                   quat[1], quat[2], quat[3], quat[0]  # QX, QY, QZ, QW
                                   )
            if sol is None:
                print "No Solution...Retrying"
                count += 1
            else:
                print "IK Solution Found"
                return list(sol)

        return sol

    def set_DOFs_for_IK_solution(self, solution):
        with self.env:
            try:
                self.robot.SetActiveDOFValues(solution)
            except:
                pass

    def get_urdf_string(self):
        with open(Config.ROBOT_URDF, 'r') as file:
            urdf_str = file.read()

        return urdf_str

    def get_next_point(self, cur_pose):
        euler = axisAngleFromMatrix (cur_pose)
        t0 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, 0, 0, 0)
        t1 = self.simulator.get_matrix_from_pose(1, 0, 0, 0, self.step_size, 0, 0)

        tpgp_wrt_gp = t0.dot(t1)
        next_pose = cur_pose.dot(tpgp_wrt_gp)

        pose = poseFromMatrix(next_pose)
        quat = pose[:4]
        trans = pose[4:7]

        urdf_str = self.get_urdf_string()
        ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
        seed_state = [0.0] * ik_solver.number_of_joints
        solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
        count = 0.0
        while True:
            if solution is not None:
                with self.simulator.env:
                    self.robot.SetActiveDOFValues(solution)
                    report = self.get_colliding_objects(self.simulator.env, None)
                    if len(report) == 0:
                        return next_pose
                    else:
                        seed_state = [count] * ik_solver.number_of_joints
                        solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
            else:
                seed_state = [count] * ik_solver.number_of_joints
                solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
            count+=1.0

    def generate_function_err_free(self):
        k = 0
        while True:
            if k > 3:
                raise StopIteration
            cpose = self.current_pose
            dof_list = []
            # Make sure first dof corresponds to initial_pose robot dof
            cur_dof = self.robot.GetActiveDOFValues()
            dof_list.append(cur_dof)
            env = self.simulator.env
            for i in range(self.number_of_values - 1):
                epose = self.get_next_point(cpose)
                self.pgp_generator_fn(epose)
                cur_dof = self.robot.GetActiveDOFValues()
                dof_list.append(cur_dof)
                cpose = epose
            # Make sure final dof corresponds to goal_pose robot dof
            epose = self.goal_pose
            self.pgp_generator_fn(epose)
            cur_dof = self.robot.GetActiveDOFValues()
            dof_list.append(cur_dof)
            if np.allclose(epose, self.goal_pose):
                print("PGP to G dof points verified")
            else:
                print("PGP to G MP Failed")
                raise StopIteration
            combined_traj = openravepy.RaveCreateTrajectory(env, '')
            combined_traj.Init(self.robot.GetActiveConfigurationSpecification())
            i = 0
            for dof in dof_list:
                combined_traj.Insert(i, dof)
                i += 1
            k += 1
            openravepy.planningutils.SmoothActiveDOFTrajectory(combined_traj, self.robot)
            print("Smooth Trajectory Created")
            yield combined_traj.serialize()

    def pgp_generator_fn(self, goal_pose):
        robot = self.simulator.env.GetRobot(Config.ROBOT_NAME)
        cur_dof = robot.GetActiveDOFValues()

        pose = poseFromMatrix(goal_pose)
        quat = pose[:4]
        trans = pose[4:7]

        urdf_str = self.get_urdf_string()
        ik_solver = self.set_IK_solver('world', 'gripper_l_base', urdf_str)
        seed_state = [0.0] * ik_solver.number_of_joints
        solution = self.get_IK_solution(ik_solver, seed_state, trans, quat)
        if solution is not None:
            with self.simulator.env:
                self.robot.SetActiveDOFValues(solution)
                report = self.get_colliding_objects(self.simulator.env, None)
                if len(report) == 0:
                    self.robot.SetActiveDOFValues(solution)



    def get_next(self, extra):
        return self.generate_function_state_err_free.next()

    def get_colliding_objects(self, env, plank):
        collision_checker = RaveCreateCollisionChecker(env, "fcl_")
        collision_checker.SetCollisionOptions(CollisionOptions.Contacts)
        env.SetCollisionChecker(collision_checker)

        robot = env.GetRobots()[0]
        bodies = env.GetBodies()
        colliding_objects = []
        for body in bodies:
            if body.IsRobot():
                if not body.CheckSelfCollision():
                    pass
                else:
                    print "Robot in self collision"
                    colliding_objects.append(body.GetName())
            elif body.GetName() != plank:
                if env.CheckCollision(robot, body):
                    colliding_objects.append(body.GetName())
        return colliding_objects

