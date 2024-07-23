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
        env = simulator.env
        robot = env.GetRobot(other_generated_values['robot'])
        plank_name = other_generated_values['obj']
        if Config.LOOPED_RUNS:
            plank_name = Config.correct_plank_name(plank_name)
        plank_body = env.GetKinBody(plank_name)
        taskmanip = interfaces.TaskManipulation(robot)
        if value == "open":
            robot.Release(plank_body)

    def execute(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        env = simulator.env
        robot = env.GetRobot(other_generated_values['robot'])
        plank_name = other_generated_values['obj']
        if Config.LOOPED_RUNS:
            plank_name = Config.correct_plank_name(plank_name)
        plank_body = env.GetKinBody(plank_name)
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "open":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.ReleaseFingers(movingdir=[1])
                else:
                    taskmanip.ReleaseFingers()
                robot.Release(plank_body)
            robot.WaitForController(0)