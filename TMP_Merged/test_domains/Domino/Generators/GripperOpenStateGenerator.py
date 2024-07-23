from src.DataStructures.Generator import Generator
import copy

class GripperOpenStateGenerator(Generator):
    def __init__(self, ll_state=None, known_argument_values=None):
        super(GripperOpenStateGenerator, self).__init__(known_argument_values, required_values=None)
        self.known_argument_values = known_argument_values
        self.generate_function_state = self.generate_function()
        self.number_of_values = 1
        # self.type = Generator.TYPE_QUERY_GENERATOR

    def reset(self):
        self.generate_function_state = self.generate_function()

    def get_next(self,flag):
        return self.generate_function_state.next()

    def generate_function(self):
        for s in ['open']:
            yield s
