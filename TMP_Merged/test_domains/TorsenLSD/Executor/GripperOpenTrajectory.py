import Config
from openravepy import *
from src.DataStructures.ArgExecutor import ArgExecutor

class GripperOpenTrajectory(ArgExecutor):

    def __init__(self,argument_name):
        super(GripperOpenTrajectory,self).__init__(argument_name)
        self.body_name_dict = Config.body_name_dict

    def __deepcopy__(self, memodict={}):
        return GripperOpenTrajectory(self.argumnet_name)

    def apply(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        env = simulator.env
        robot = env.GetRobot(other_generated_values['robot'])
        part_name = other_generated_values['part']
        part_body = env.GetKinBody(self.body_name_dict[part_name])
        taskmanip = interfaces.TaskManipulation(robot)
        if value == "open":
            robot.Release(part_body)

    def execute(self,low_level_state,value,other_generated_values):
        simulator = low_level_state.simulator
        env = simulator.env
        robot = env.GetRobot(other_generated_values['robot'])
        part_name = other_generated_values['part']
        part_body = env.GetKinBody(self.body_name_dict[part_name])
        taskmanip = interfaces.TaskManipulation(robot)
        state = value
        if state == "open":
            with robot:
                if robot.GetName() == 'yumi':
                    taskmanip.ReleaseFingers(movingdir=[1])
                else:
                    taskmanip.ReleaseFingers()
                robot.Release(part_body)
            robot.WaitForController(0)
