import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class GripperCloseTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(GripperCloseTrajectory,self).__init__(argument_name)

    def __deepcopy__(self, memodict={}):
        return GripperCloseTrajectory(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        robot = low_level_state.simulator.env.GetRobot(other_generated_values['robot'])
        taskmanip = interfaces.TaskManipulation(robot)

        # if robot.GetName() != 'fetch':
        #     import IPython
        #     IPython.embed()

        if value == "close":
            # robot.Grab(low_level_state.simulator.env.GetKinBody(self.body_name_dict[other_generated_values["part"]]), robot.GetActiveManipulator().GetEndEffector())
            robot.Grab(low_level_state.simulator.env.GetKinBody(other_generated_values["domino"]))

    def execute(self,low_level_state,value,other_generated_values):
        robot = low_level_state.simulator.env.GetRobot(other_generated_values['robot'])
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "close":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.CloseFingers(movingdir=[-1])
                else:
                    taskmanip.CloseFingers()
                robot.Grab(low_level_state.simulator.env.GetKinBody(other_generated_values["domino"]))
            robot.WaitForController(0)
