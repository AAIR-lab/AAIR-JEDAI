from src.DataStructures.Generator import Generator
import Config
import openravepy

class CurrentBasePoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(CurrentBasePoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR
        self.known_argument_values = known_argument_values

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            # import IPython
            # IPython.embed()
            robot = self.simulator.env.GetRobot(self.known_argument_values["robot"])
            t = robot.GetTransform()
            x = t[0,3]
            y = t[1,3]
            theta = openravepy.axisAngleFromRotationMatrix(t)[2]
            yield [x,y,theta]

    def get_next(self,flag):
        return self.generate_function_state.next()

