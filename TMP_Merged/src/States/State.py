class State(object):
    # State base class

    def __init__(self, universe, functions, values):
        '''
        This class defines base class for states.
        Inherited By : HLState, LLState

        Inputs :
        univers : list of objects ( object of the Object)
        functions : list of functions ( object of the Function)

        '''

        self.universe = universe
        self.list_functions = functions
        self.values = values

