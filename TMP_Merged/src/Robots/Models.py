import openravepy
import Config
import src.util as util
import numpy
import copy
import numpy as np
from openravepy import *
from src.Robots import *

class FetchConfig(object):
    def __init__(self):
        self.jointnames = (["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                            "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint" ])

        self.active_manipulators = {'arm_and_torso': 'arm_torso'}
        # self.files = {'model-srdf': '/res/models/openrave/openrave-fetch.srdf',
        #               'model-urdf': '/res/models/openrave/openrave-fetch.urdf'}
        self.armTuckDOFs = [0.00000000e+00, 1.71996798e+00, 2.70622649e-06,
                    0.00000000e+00, 0.00000000e+00, 4.83987850e-16,
                    0.00000000e+00, 7.51135265e-16, 0.00000000e+00,
                    1.39998343e+00, 1.32000492e+00, 0.00000000e+00,
                    -1.99845334e-01, 1.66000647e+00, -1.34775426e-06]

        self.armSecureDOFs = [0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565 , 0 , 0 ]


class FetchOpenRaveRobotModel:
    def __init__(self, environment, doMapJoints=True):
        env = environment
        config = FetchConfig()
        fetch_urdf = Config.FETCH_URDF
        fetch_srdf = Config.FETCH_SRDF
        module = openravepy.RaveCreateModule(env, 'urdf')
        with env:
            util.set_paths()
            name = module.SendCommand('load ' + fetch_urdf + ' ' + fetch_srdf)
            util.reset_paths()
        with env:
            body = env.GetKinBody(name)
            self.robot = env.GetRobot('fetch')
            self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in config.jointnames])
            self.robot.SetActiveManipulator(config.active_manipulators['arm_and_torso'])
            self.initGripper()
            self.setJointAcclerationLimits(2)
            self.robot.SetAffineTranslationMaxVels([10.5,10.5,10.5])
            self.robot.SetAffineRotationAxisMaxVels(np.ones(4))

        if (doMapJoints):
            openravepy.raveLogInfo("Mapping physical robot joints")
            self.mapPhysicalRobotJointValues()
        else:
            self.robot.SetDOFValues(numpy.asarray(config.armTuckDOFs))

    def initGripper(self):
        """Setup gripper closing direction and tool direction """
        gripperManip = self.robot.GetActiveManipulator()
        gripperIndices = gripperManip.GetGripperIndices()
        closingDirection = numpy.zeros(len(gripperIndices))

        for i, gi in enumerate(gripperIndices):
            closingDirection[i] = -1.

        gripperManip.SetChuckingDirection(closingDirection)

        gripperManip.SetLocalToolDirection([1, 0, 0])

    def mapPhysicalRobotJointValues(self):
        import pickle
        joint_dump = open(Config.MISC_DIR+"/real_world_exec/fetch/init_joint_state.pickle","rb")
        joint_poses = pickle.load(joint_dump)
        joint_dump.close()
        robot_joints = ['torso_lift_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
                        'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        vals = [2,6,7,8,9,10,11,12]
        self.robot.SetActiveDOFs([self.robot.GetJoint(joint).GetDOFIndex() for joint in robot_joints])
        self.robot.SetActiveDOFValues([v for i,v in enumerate(joint_poses[0]) if i in vals])

    def setJointAcclerationLimits(self, val):
        accel_limits = self.robot.GetDOFAccelerationLimits()
        manipulator = self.robot.GetActiveManipulator()
        accel_limits[manipulator.GetArmIndices()] = [val] * manipulator.GetArmDOF()
        self.robot.SetDOFAccelerationLimits(accel_limits)

    def getRobot(self):
        return self.robot

class UAVOpenRaveRobotModel:
    def __init__(self,env):
        robot = env.ReadRobotXMLFile(Config.UAV_XML)
        robot.SetName(Config.ROBOT_NAME)
        t = [1,0,0,0,13.5,6,1.7]
        robot.GetLink('base_link').SetTransform(t)
        with env:
            env.AddRobot(robot)
        # robot.SetActiveDOFs([], openravepy.DOFAffine.X | openravepy.DOFAffine.Y | openravepy.DOFAffine.Z ,[0,0,0])
        robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in Config.ROBOT_BASE_JOINTS])


class YuMiConfig(object):
    def __init__(self):
        self.left_arm_jointnames = (["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l", "yumi_joint_4_l",
				 "yumi_joint_5_l", "yumi_joint_6_l"])
        self.right_arm_jointnames = (["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r", "yumi_joint_4_r",
				 "yumi_joint_5_r", "yumi_joint_6_r"])

        self.active_manipulators = {'left_arm_effector': 'left_arm_effector', 'right_arm_effector': 'right_arm_effector'}

        self.armTuckDOFs = [0.1072470542163906, -2.372612683231338, 2.356246130343412, 0.5157132610529159, 0.23051264210714933, 0.7072600576461908, -0.1937667520633811]

        self.armSecureDOFs = [0.1072470542163906, -2.372612683231338, 2.356246130343412, 0.5157132610529159, 0.23051264210714933, 0.7072600576461908, -0.1937667520633811]

        # self.left_arm_tuck_DOFs = [0.1072470542163906, -2.372612683231338, 2.356246130343412, 0.5157132610529159, 0.23051264210714933, 0.7072600576461908, -0.1937667520633811]
        #self.right_arm_tuck_DOFs = [-0.10264738615826983, -2.3646874727520912, -2.3558966908535153, 0.516495984426138, -0.21444955923116824, 0.7058787229067812, 0.1795185371549159]

        # self.left_arm_tuck_DOFs = [0.32592421901965096, -2.34200605696553, 2.2443831554480718, 0.6143840852897609, 0.4095033675810835, 0.8291095083790417, -0.5451207814487959]
        self.left_arm_tuck_DOFs = [ 0.00000000e+00, -2.26875349e+00, 2.35601996e+00, 5.23773309e-01,  0.00000000e+00, 6.99004365e-01, -1.74532925e-04]
        # self.left_arm_tuck_DOFs = [-0.79098322, -1.78250477, -0.7707374, 0.23684118, -0.66654124, 0.68870692, -0.0001745329]
        self.right_arm_tuck_DOFs = [-0.3264493983797501, -2.337921596190979, -2.2443558819818707, 0.6147207056518011, -0.40258980361318003, 0.8277095613394793, 0.538388770740175]

class YuMiOpenRaveRobotModel:
    def __init__(self, environment, doMapJoints=True):
        self.env = environment
        self.yumi_config = YuMiConfig()
        yumi_urdf = Config.YUMI_URDF
        yumi_srdf = Config.YUMI_SRDF
        module = openravepy.RaveCreateModule(self.env, 'urdf')
        with self.env:
            util.set_paths()
            name = module.SendCommand('load ' + yumi_urdf + ' ' + yumi_srdf)
            util.reset_paths()
            body = self.env.GetKinBody(name)
            self.robot = self.env.GetRobots()[0]
            self.set_right_arm()
            self.set_left_arm()

            if Config.HAND == "left":
                self.robot.SetActiveDOFs([self.robot.GetJoint().GetDOFIndex() for name in self.yumi_config.left_arm_jointnames])
                self.robot.SetActiveManipulator(self.yumi_config.active_manipulators['left_arm_effector'])
            else:
                self.robot.SetActiveDOFs(
                    [self.robot.GetJoint(name).GetDOFIndex() for name in self.yumi_config.right_arm_jointnames])
                self.robot.SetActiveManipulator(self.yumi_config.active_manipulators['right_arm_effector'])
            self.robot.SetTransform(np.eye(4))
            self.initGripper()
            self.setJointAcclerationLimits(2)
        self.env.UpdatePublishedBodies()

        if (doMapJoints):
            openravepy.raveLogInfo("Mapping physical robot joints")
            self.mapPhysicalRobotJointValues()
        # else:
        #     # import IPython
        #     # IPython.embed()
        #     self.robot.SetDOFValues(numpy.asarray(config.armTuckDOFs))



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

    def getRobot(self):
        return self.robot

    def set_left_arm(self):
        # manip = self.robot.SetActiveManipulator(self.yumi_config.active_manipulators['left_arm_effector'])
        joint_names = ["yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
                           "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"]
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in joint_names])
        solution = self.yumi_config.left_arm_tuck_DOFs
        with self.env:
            try:
                self.robot.SetActiveDOFValues(solution)
            except:
                pass

    def set_right_arm(self):
        # manip = self.robot.SetActiveManipulator(self.yumi_config.active_manipulators['right_arm_effector'])
        joint_names = ["yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
                           "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"]
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in joint_names])
        solution = self.yumi_config.right_arm_tuck_DOFs
        with self.env:
            try:
                self.robot.SetActiveDOFValues(solution)
            except:
                pass
