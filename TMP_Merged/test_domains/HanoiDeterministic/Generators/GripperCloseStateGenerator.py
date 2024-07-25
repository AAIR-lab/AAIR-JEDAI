from src.DataStructures.Generator import Generator
import copy


class GripperCloseStateGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(GripperCloseStateGenerator, self).__init__(known_argument_values, required_values=None)
        self.known_argument_values = known_argument_values
        next_hl_state_true_prop = known_argument_values["next_hl_state"].getTrueProps()
        self.generate_function_state = self.generate_function()
        self.number_of_values = 1
        # self.type = Generator.TYPE_QUERY_GENERATOR

    def reset(self):
        self.generate_function_state = self.generate_function()

    def get_next(self,flag):
        return self.generate_function_state.next()

    def generate_function(self):
        for s in ["close"]:
            yield s
