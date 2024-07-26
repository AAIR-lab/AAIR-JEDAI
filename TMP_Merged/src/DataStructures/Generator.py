class Generator(object):
    TYPE_QUERY_GENERATOR = 'type_query_generator'

    def __init__(self, known_argument_values=None, required_values=None):
        if required_values is not None:
            available_values = set(required_values) & set(known_argument_values.keys())
            assert available_values == set(required_values), str(self.__class__) + " requires " + str(required_values)+" but only found "+ str(available_values)
        self.is_configured = False
        self.last_generated_value = None
        self.type = None
        self.dofs = 8 #TODO, fetch arm manipulator specific
        self.dependencies = []

    def configure(self, map=None):
        self.is_configured = True

    def reset(self):
        self.is_configured = False

    def store(self, value):
        self.last_generated_value = value

    def get_current(self):
        return self.last_generated_value

    def get_next(self,flag):
        raise NotImplementedError
