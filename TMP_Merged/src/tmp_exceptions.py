class OpenRaveSimulatorException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return self.message



class FailedPredicateException(Exception):
    def __init__(self, predicate, last_successful_state=None, failure_reason=[]):
        self.predicate = predicate
        self.last_successful_state = last_successful_state
        self.failure_reason = failure_reason

    def __str__(self):
        return str(self.predicate)

class OutOfPossibleErrorsException(Exception):
    def __init__(self,failure_node):
        super(OutOfPossibleErrorsException,self).__init__()
        self.failure_node = failure_node

class TimeOutException(Exception):
    def __init__(self,hl_action_node=None):
        super(TimeOutException,self).__init__()
        self.hl_action_node = hl_action_node

