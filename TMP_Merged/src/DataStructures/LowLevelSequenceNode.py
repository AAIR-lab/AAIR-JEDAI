from src.DataStructures.GraphNode import GraphNode
import copy


class LowLevelSequenceNode(GraphNode):
    def __init__(self, ll_action_spec, debug_name=''):
        super(LowLevelSequenceNode, self).__init__()
        self.ll_action_spec = ll_action_spec
        self.children = None
        self.debug_name = debug_name
        self.generator = None
        self.arg = None
        self.depends_on = []
        self.dependees = []
        self.__parent = None
        self.position_int = None

    def __deepcopy__(self, memodict={}):
        ll_action_spec_cpy = copy.deepcopy(self.ll_action_spec)
        debug_cpy = copy.deepcopy(self.debug_name)
        arg_cpy = copy.deepcopy(self.arg)
        gen_cpy = copy.deepcopy(self.generator)
        lldn = LowLevelSequenceNode(ll_action_spec=ll_action_spec_cpy, debug_name=debug_cpy)
        lldn.set_generator(gen_cpy)
        lldn.depends_on = copy.deepcopy(self.depends_on)
        lldn.dependees = copy.deepcopy(self.dependees)
        lldn.position_int = copy.deepcopy(self.position_int)
        lldn.arg = arg_cpy
        return lldn

    def set_value_prestine(self):
        self.__value_dirty = False

    def set_parent(self, parent):
        self.__parent = parent

    def set_children(self, children):
        self.children = children

    def add_child(self, child):
        if self.children is None:
            self.children = []

        self.children.append(child)

    def set_generator(self, generator):
        self.generator = generator

    def reset_generator(self):
        self.generator.reset()

    def configure_generator(self, hl_args, ll_state ):
        self.generator.configure(hl_args, ll_state)

    def is_generator_configured(self):
        return self.generator.is_configured

    def get_current_value(self):
        return self.generator.get_current()

    def get_new_value(self, required_values):
        return self.generator.get_next(required_values)

    def get_parent(self):
        return self.__parent

    def get_children(self):
        return self.children

    def get_child(self):
        if self.children is not None and len(self.children) >=1:
            return self.children[0]
        else:
            return None
