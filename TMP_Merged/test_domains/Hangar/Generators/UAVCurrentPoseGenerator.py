from src.DataStructures.Generator import Generator
import Config

class UAVCurrentPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(UAVCurrentPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            robot = self.simulator.env.GetRobot(Config.ROBOT_NAME)
            yield robot.GetLink('base_link').GetTransform()

    def get_next(self,flag):
        return self.generate_function_state.next()

