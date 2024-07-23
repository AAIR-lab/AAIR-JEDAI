from src.DataStructures.Generator import Generator
import Config

class HoldPoseGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(HoldPoseGenerator, self).__init__(known_argument_values=known_argument_values, required_values=None)
        self.generate_function_state = self.generate_function()
        self.simulator = ll_state.simulator
        self.type = Generator.TYPE_QUERY_GENERATOR
        self.known_argument_values = known_argument_values

    def reset(self):
        self.generate_function_state = self.generate_function()

    def generate_function(self):
        while True:
            if self.known_argument_values['robot'] == 'yumi':
                if self.known_argument_values['hand'] == 'left':
                    dof_values = self.simulator.robots['yumi'].left_arm_tuck_DOFs
                else:
                    dof_values = self.simulator.robots['yumi'].right_arm_tuck_DOFs
            else:
                # fetch holding pose with arm in front
                dof_values = [0.15, 0.72466344, -0.05064385, -1.73952133, 2.25099986, -1.50486781, -0.02543545, 2.76926565 ]
            
            yield dof_values

    def get_next(self,flag):
        return self.generate_function_state.next()