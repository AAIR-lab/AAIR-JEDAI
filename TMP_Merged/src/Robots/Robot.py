import Config
import multiprocessing
from openravepy import *
from openravepy.misc import *
from trac_ik_python.trac_ik import IK
import numpy as np
import os


class Robot(object):

    def __init__(self):
        pass
    #
    # def set_robot(self,robot):
    #     self.robot = robot
    #
    # def set_env(self,env):
    #     self.env = env
    #
    # def set_active_manip(self,manip,joint_names):
    #     self.set_active_dofs(joint_names)
    #     self.robot.SetActiveManipuolator(manip)
    #
    # def set_active_dofs(self,joint_names):
    #     self.robot.SetActiveDOFs([self.robot.GetJoint().GetDOFIndex()] for name in joint_names)
    #
    # def get_active_dofs(self):
    #     return self.robot.GetActiveDOFs()
    #
    #
    # def get_active_manip(self,manip):
    #     return self.robot.GetActiveManipulator()
    #
    # def get_current_dof(self):
    #     return self.robot.GetActiveDOFValues()
    #
    # def set_dof(self,dof):
    #     return self.robot.SetActiveDOFValues(dof)




