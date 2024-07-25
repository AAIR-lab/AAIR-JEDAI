class Action(object):
    def __init__(self, name, precondition_pos, precondition_neg, effect_pos, effect_neg, args = []):
        self.name = name
        self.precondition_pos = precondition_pos
        self.precondition_neg = precondition_neg
        self.effect_pos = effect_pos
        self.effect_neg = effect_neg
        self.arg_list = args

    def __repr__(self):
        return str(self.name) +": Args: " + str(self.arg_list)

