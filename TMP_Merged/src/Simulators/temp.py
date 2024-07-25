import openravepy
import Config
from src.Robots.Models import FetchConfig
import time


env = openravepy.Environment()
env.Load("/home/naman/TMP_Merged/GeneratedEnvironments/can_world_100_cans.dae")
env.SetViewer('rviz')

module = openravepy.RaveCreateModule(env,'urdf')
with env:
    name = module.SendCommand('load ' + Config.ROBOT_URDF +' ' + Config.ROBOT_SRDF)


config = FetchConfig()

with env:
    robot = env.GetRobot(name)
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in config.jointnames])
    robot.SetActiveManipulator(config.active_manipulators['arm_and_torso'])

robot.SetActiveDOFValues(config.armSecureDOFs)

non_movable_bodies = ['table6']




for i in range(10):
    print "Iteration ",i
    print "Removing Cans"
    ot_map = {}
    with env:
        for body in env.GetBodies():
            if not body.IsRobot() and body.GetName() not in non_movable_bodies:
                body_name = body.GetName()
                body_transform = body.GetTransform()
                ot_map[body_name] = {'object':body , 'transform' : body_transform}
                env.Remove(body)
    # time.sleep(1)
    with env:
        for body_name in ot_map:
            env.AddKinBody(ot_map[body_name]["object"])
    print "Added back the cans"

env.Destroy()