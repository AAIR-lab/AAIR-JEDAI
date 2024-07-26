import Config
from src.Robots.Robot import Robot
from openravepy import *
import src.util as util
import numpy as np
from trac_ik_python.trac_ik import IK
import multiprocessing
import os
class YumiRobot(object):
    def __init__(self,env, doMapJoints=False):
        #unzipping env files
        if not os.path.isdir(Config.MISC_DIR+'RobotModels'):
            import tarfile
            my_tar = tarfile.open(Config.MISC_DIR+'RobotModels.tar.gz')
            my_tar.extractall(Config.MISC_DIR)
            my_tar.close()
        self.env = env
        self.robot = None
        self.right_arm_joints = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
                           "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"]
        self.left_arm_joints = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
                           "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"]
        self.yumi_urdf = Config.YUMI_URDF
        self.yumi_srdf = Config.YUMI_SRDF
        self.urdf_str = self.get_urdf_string()
        self.left_arm_tuck_DOFs = [ 0., -2.26892803, 2.35619449, 0.52359878,  0. ,  0.6981317 , -0. ]
        self.right_arm_tuck_DOFs = [  0.00000000e+00,  -2.26892803e+00, -2.35619449e+00,  5.23947841e-01, 5.23598776e-04,   6.76489618e-01,  -1.74532925e-04 ]
        
        module = RaveCreateModule(self.env, 'urdf')
        with self.env:
            util.set_paths()
            name = module.SendCommand('loadURI ' + self.yumi_urdf + ' ' + self.yumi_srdf)
            util.reset_paths()

        body = self.env.GetKinBody(name)
        self.robot = self.env.GetRobot(name)
        self.set_right_arm()
        self.set_left_arm()

        self.robot.SetTransform(np.eye(4))
        self.initGripper()
        self.setJointAcclerationLimits(2)
        self.env.UpdatePublishedBodies()
        # import IPython
        # IPython.embed()
        self.left_ik_solver = IK("world","gripper_l_base",urdf_string=self.urdf_str)
        self.right_ik_solver = IK("world","gripper_r_base",urdf_string=self.urdf_str)
        self.left_arm_objects = []
        self.right_arm_objects = []
        self.execute_left_arm_objects = []
        self.execute_right_arm_objects = []
        self.previous_arm = None


    def get_urdf_string(self):
        util.set_paths()
        with open(self.yumi_urdf, 'r') as file:
            urdf_str = file.read()
        util.reset_paths()
        return urdf_str


    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.robot.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = np.zeros(len(gripperIndices))

    def setJointAcclerationLimits(self, val):
        accel_limits = self.robot.GetDOFAccelerationLimits()
        manipulator = self.robot.GetActiveManipulator()
        accel_limits[manipulator.GetArmIndices()] = [val] * manipulator.GetArmDOF()
        self.robot.SetDOFAccelerationLimits(accel_limits)


    def set_left_arm(self):
        self.activate_left_arm()
        solution = self.left_arm_tuck_DOFs
        with self.env:
            try:
                self.robot.SetActiveDOFValues(solution)
            except:
                pass
        self.openGrippers()

    def set_right_arm(self):
        self.activate_right_arm()
        solution = self.right_arm_tuck_DOFs
        with self.env:
            try:
                self.robot.SetActiveDOFValues(solution)
            except:
                pass
        self.openGrippers()

    def activate_arm(self,arm):
        if arm not in self.robot.GetActiveManipulator().GetName():
            if arm == "left":
                grabbed_objects = self.robot.GetGrabbed()
                for obj in grabbed_objects:
                    self.right_arm_objects.append(obj.GetName())
                self.robot.ReleaseAllGrabbed()
                self.activate_left_arm()
                for obj in self.left_arm_objects:
                    self.robot.Grab(self.env.GetKinBody(obj))
                self.left_arm_objects = []
            else:
                grabbed_objects = self.robot.GetGrabbed()
                for obj in grabbed_objects:
                    self.left_arm_objects.append(obj.GetName())
                self.robot.ReleaseAllGrabbed()
                self.activate_right_arm()
                for obj in self.right_arm_objects:
                    self.robot.Grab(self.env.GetKinBody(obj))
                self.right_arm_objects = []
            self.previous_arm = arm

    def execute_activate_arm(self,arm):
        if arm not in self.robot.GetActiveManipulator().GetName():
            if arm == "left":
                grabbed_objects = self.robot.GetGrabbed()
                for obj in grabbed_objects:
                    self.execute_right_arm_objects.append(obj.GetName())
                self.robot.ReleaseAllGrabbed()
                self.activate_left_arm()
                for obj in self.execute_left_arm_objects:
                    self.robot.Grab(self.env.GetKinBody(obj))
                self.execute_left_arm_objects = []
            else:
                grabbed_objects = self.robot.GetGrabbed()
                for obj in grabbed_objects:
                    self.execute_left_arm_objects.append(obj.GetName())
                self.robot.ReleaseAllGrabbed()
                self.activate_right_arm()
                for obj in self.execute_right_arm_objects:
                    self.robot.Grab(self.env.GetKinBody(obj))
                self.execute_right_arm_objects = []
            self.previous_arm = arm


    def activate_left_arm(self):
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.left_arm_joints])
        self.robot.SetActiveManipulator("left_arm_effector")

    def activate_right_arm(self):
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.right_arm_joints])
        self.robot.SetActiveManipulator("right_arm_effector")

    def openGrippers(self):
        taskmanip = interfaces.TaskManipulation(self.robot)
        with self.robot:
            taskmanip.ReleaseFingers(movingdir=[1])
        self.robot.WaitForController(0)

    def get_ik_solutions(self,end_effector_solution,check_collisions=False):
        if str(self.robot.GetActiveManipulator().GetName()) == "right_arm_effector":
            return self.compute_ik_solutions(end_effector_solution,self.right_ik_solver,check_collisions)
        else:
            return self.compute_ik_solutions(end_effector_solution,self.left_ik_solver,check_collisions)

    def get_IK_solution(self, ik_solver, seed_state, trans, quat):
        solution = None
        try:
            solution = ik_solver.get_ik(seed_state,
                                    trans[0], trans[1], trans[2],  # X, Y, Z
                                    quat[1], quat[2], quat[3], quat[0]  # QX, QY, QZ, QW
                                    )
        except:
            pass

        if solution is None:
            return None
        else:
            return list(solution)


    def compute_ik_solutions(self,end_effector_transform,ik_solver,check_collisions=False):
        end_effector_pose = poseFromMatrix(end_effector_transform)
        quat = end_effector_pose[:4]
        trans = end_effector_pose[4:7]
        solutions = dict()
        for i in range(Config.MAX_IKs_TO_CHECK_FOR_MP):
            seed_state = [np.random.uniform(0, 100)] * ik_solver.number_of_joints
            sol = self.get_IK_solution(ik_solver, seed_state, trans, quat)
            if sol is not None:
                solutions[i] = sol
        if check_collisions:
            for i in solutions.keys():
                solution = solutions[i]
                with self.env:
                    bodies = self.env.GetBodies()
                    last_DOF_state = self.robot.GetActiveDOFValues()
                    self.robot.SetActiveDOFValues(solution)
                    collisions = []
                    for body in bodies:
                        if body.IsRobot():
                            collisions.append(body.CheckSelfCollision())
                        else:
                            collisions.append(self.env.CheckCollision(self.robot, body))
                    self.robot.SetActiveDOFValues(last_DOF_state)
                    colliding_objects = list(np.asarray(bodies)[np.asarray(collisions)])
                    colliding_objects = [body for body in colliding_objects if body not in self.robot.GetGrabbed()]
                    if len(colliding_objects) != 0:
                        # print("Detected Collision with: "+str(colliding_objects))
                        del solutions[i]
        elif self.robot.CheckSelfCollision():
            del solutions[i]
        return solutions.values()





