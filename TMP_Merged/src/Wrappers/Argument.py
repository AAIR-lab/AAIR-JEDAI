class Argument(object):

    def __init__(self, name, type, value=None):
        self.name = name
        self.type = type
        self.value = value

    def __repr__(self):
        return self.name + ":" + self.type + ":" + str(self.value)
