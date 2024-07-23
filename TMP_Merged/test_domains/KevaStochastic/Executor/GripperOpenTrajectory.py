import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class GripperOpenTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(GripperOpenTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return GripperOpenTrajectory(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        robot = simulator.env.GetRobot(Config.ROBOT_NAME)
        taskmanip = interfaces.TaskManipulation(robot)
        if value == "open":
            robot.ReleaseAllGrabbed()

    def execute(self,low_level_state,value,other_generated_values,
                sleep_interval_in_ms=1):
        
        simulator = low_level_state.simulator
        robot = simulator.env.GetRobot(Config.ROBOT_NAME)
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "open":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.ReleaseFingers(movingdir=[1])
                else:
                    taskmanip.ReleaseFingers()
                robot.ReleaseAllGrabbed()

        return self.wait_for_controller(robot, low_level_state,
                                        sleep_interval_in_ms)
