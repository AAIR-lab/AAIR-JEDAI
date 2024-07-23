from src.Robots.Robot import Robot
import Config
from openravepy import *


class UAVRobot(Robot):
    def __init__(self,env,real_robot):

        self.robot = env.ReadRobotXMLFile(Config.MISC_DIR+"RobotModels/UAV/robot.xml")
        self.robot.SetName("uav")
        t = [1,0,0,0,13.5,6,1.7]
        self.robot.SetTransform(t)
        with env:
            env.AddRobot(self.robot)
        self.activate_base_joints()

    def activate_base_joints(self):
        self.robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.Z ,[0,0,0])
        # robot.SetActiveDOFs([robot.GetJoint(joint).GetDOFIndex() for joint in Config.ROBOT_BASE_JOINTS])
