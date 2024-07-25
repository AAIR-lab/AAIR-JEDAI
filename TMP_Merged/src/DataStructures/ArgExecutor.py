class ArgExecutor(object):
    def __init__(self,argument_name):
        self.argumnet_name = argument_name


    def execute(self,low_level_state,value,other_generated_values):
        raise NotImplementedError

    def apply(self,low_level_state,value,other_generated_values):
        raise NotImplementedError
