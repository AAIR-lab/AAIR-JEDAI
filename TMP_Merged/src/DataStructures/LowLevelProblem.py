class LowLevelProblem:
    def __init__(self, hl_args, generated_value, generated_value_type, arg, dofs=None):
        self.hl_args = hl_args
        self.generated_value = generated_value
        self.generated_value_type = generated_value_type
        self.arg = arg
        self.dofs = dofs

    def __repr__(self):
        return  str(self.hl_args)+ " " + str(self.generated_value_type)  + " "+ str(self.generated_value)
