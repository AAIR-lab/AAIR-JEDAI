import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class ManipTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(ManipTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return ManipTrajectory(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        robot = simulator.env.GetRobot(other_generated_values['robot'])
        # with robot:
        traj = RaveCreateTrajectory(simulator.env, '')
        Trajectory.deserialize(traj, value)
        # robot.GetController().SetPath(traj)
        # import IPython
        # IPython.embed()
        numWayPoints = traj.GetNumWaypoints()
        lastWayPoint = traj.GetWaypoint(numWayPoints - 1)
        lastWayPointDOFs = simulator.get_joint_values_from_waypoint(lastWayPoint, robot)

        robot.SetActiveDOFValues(lastWayPointDOFs)
        pass

    def execute(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        robot = simulator.env.GetRobot(other_generated_values['robot'])
        traj = RaveCreateTrajectory(simulator.env, '')
        Trajectory.deserialize(traj, value)
        with robot:
            robot.GetController().SetPath(traj)
        robot.WaitForController(0)
        pass