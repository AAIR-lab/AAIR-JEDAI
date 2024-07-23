import copy
class Precondition(object):
    def __init__(self, list_predicates):
        self.predicates = list_predicates

    def __deepcopy__(self, memodict={}):
        return Precondition(copy.deepcopy(self.predicates))

    def get_predicates(self):
        return self.predicates
