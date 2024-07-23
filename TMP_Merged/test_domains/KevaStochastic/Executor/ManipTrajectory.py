import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor
import time

class ManipTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(ManipTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return ManipTrajectory(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        robot = simulator.env.GetRobot(Config.ROBOT_NAME)
        # with robot:
        traj = RaveCreateTrajectory(simulator.env, '')
        traj.Init(robot.GetActiveConfigurationSpecification('linear'))
        Trajectory.deserialize(traj, value)
        # robot.GetController().SetPath(traj)
        # import IPython
        # IPython.embed()
        numWayPoints = traj.GetNumWaypoints()
        lastWayPoint = traj.GetWaypoint(numWayPoints - 1)
        lastWayPointDOFs = simulator.get_joint_values_from_waypoint(lastWayPoint, robot)

        robot.SetActiveDOFValues(lastWayPointDOFs)
        pass

    def execute(self,low_level_state,value,other_generated_values,
                sleep_interval_in_ms=1):
        simulator = low_level_state.simulator
        robot = simulator.env.GetRobot(Config.ROBOT_NAME)
        traj = RaveCreateTrajectory(simulator.env, '')
        Trajectory.deserialize(traj, value)
        
        controller = robot.GetController()
        controller.SetPath(traj)
        
        return self.wait_for_controller(robot, low_level_state,
                                        sleep_interval_in_ms)