from src.DataStructures.Generator import Generator
import Config

class InitPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(InitPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
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
            if robot.GetActiveManipulator().GetName() == "right_arm_effector":
                dof_values = [-0.3264493983797501, -2.337921596190979, -2.2443558819818707, 0.6147207056518011, -0.40258980361318003, 0.8277095613394793, 0.538388770740175]
            else:
                dof_values = [0.32592421901965096, -2.34200605696553, 2.2443831554480718, 0.6143840852897609, 0.4095033675810835, 0.8291095083790417, -0.5451207814487959]
            yield dof_values

    def get_next(self,flag):
        return self.generate_function_state.next()