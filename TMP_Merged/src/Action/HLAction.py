from src.Wrappers.Action import Action

class HLAction(Action):

    def __init__(self, action_name, precondition_pos, precondition_neg, effect_pos, effect_neg, arguments):
        super(HLAction, self).__init__(action_name, precondition_pos, precondition_neg, effect_pos, effect_neg, arguments)
        self.hl_arguments = arguments
        self.actionName = action_name

    def __repr__(self):
        return  super(HLAction, self).__repr__()
