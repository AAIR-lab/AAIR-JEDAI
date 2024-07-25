import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class BaseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(BaseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return BaseTrajectory(self.argumnet_name)

    def apply(self,ll_state,value,other_generated_values):
        if value is True:
            pass
        else:
            robot = ll_state.simulator.env.GetRobot(other_generated_values["robot"])
            simulator = ll_state.simulator
            old_active_dofs = robot.GetActiveDOFIndices()
            robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
            # traj = RaveCreateTrajectory(simulator.env, '')
            # Trajectory.deserialize(traj, value)
            # numWayPoints = traj.GetNumWaypoints()
            # lastWayPoint = traj.GetWaypoint(numWayPoints - 1)
            # lastWayPointDOFs = simulator.get_joint_values_from_waypoint(lastWayPoint, robot)
            # robot.SetActiveDOFValues(lastWayPointDOFs)
            robot.SetActiveDOFValues(other_generated_values["tbpose"])
            robot.SetActiveDOFs(old_active_dofs)
    
    def execute(self,ll_state,value,other_generated_values):
        if value is True:
            pass
        else:
            robot = ll_state.simulator.env.GetRobot(other_generated_values["robot"])
            simulator = ll_state.simulator
            old_active_dofs = robot.GetActiveDOFIndices()
            robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
            traj = RaveCreateTrajectory(simulator.env, '')
            Trajectory.deserialize(traj, value)
            with robot:
                robot.GetController().SetPath(traj)
            robot.WaitForController(0)
            robot.SetActiveDOFs(old_active_dofs)
            # traj = RaveCreateTrajectory(simulator.env, '')
            # Trajectory.deserialize(traj, value)
            # with robot:
            #     robot.GetController().SetPath(traj)
            # old_active_dofs = robot.GetActiveDOFIndices()
            # robot.SetActiveDOFValues(other_generated_values["tbpose"])
            # robot.SetActiveDOFs(old_active_dofs)
