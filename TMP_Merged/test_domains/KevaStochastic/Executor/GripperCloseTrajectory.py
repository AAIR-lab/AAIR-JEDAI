import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class GripperCloseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(GripperCloseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return GripperCloseTrajectory(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        # import IPython
        # IPython.embed()
        robot = low_level_state.simulator.env.GetRobot(Config.ROBOT_NAME)
        taskmanip = interfaces.TaskManipulation(robot)
        if value == "close":
            robot.Grab(low_level_state.simulator.env.GetKinBody(other_generated_values["obj"]))

    def execute(self,low_level_state,value,other_generated_values,
                sleep_interval_in_ms=1):
        
        robot = low_level_state.simulator.env.GetRobot(Config.ROBOT_NAME)
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "close":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.CloseFingers(movingdir=[-1])
                else:
                    taskmanip.CloseFingers()
                robot.Grab(low_level_state.simulator.env.GetKinBody(other_generated_values["obj"]))
        
        return self.wait_for_controller(robot, low_level_state,
                                        sleep_interval_in_ms)
