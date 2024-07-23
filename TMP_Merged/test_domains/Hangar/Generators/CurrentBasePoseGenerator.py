from src.DataStructures.Generator import Generator
import Config

class CurrentBasePoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(CurrentBasePoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            # import IPython
            # IPython.embed()
            robot = self.simulator.env.GetRobot(Config.ROBOT_NAME)
            dof_values = robot.GetDOFValues()
            yield [dof_values[1],dof_values[2],dof_values[3]]

    def get_next(self,flag):
        return self.generate_function_state.next()

