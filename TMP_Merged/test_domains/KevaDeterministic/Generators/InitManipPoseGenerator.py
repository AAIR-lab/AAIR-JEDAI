from src.DataStructures.Generator import Generator
import Config

class InitManipPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(InitManipPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR
        self.robot = self.simulator.robots[known_argument_values["robot"]]

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            if "left" in self.robot.robot.GetActiveManipulator().GetName():
                yield self.robot.left_arm_tuck_DOFs
            else:
                yield self.robot.right_arm_tuck_DOFs

    def get_next(self,flag):
        return self.generate_function_state.next()

