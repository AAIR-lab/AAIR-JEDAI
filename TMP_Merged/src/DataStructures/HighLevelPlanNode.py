from src.DataStructures.GraphNode import GraphNode
import copy


class HighLevelPlanNode(GraphNode):
    def __init__(self, hl_action, ll_state=None, hl_state=None,children  =[] ):
        super(HighLevelPlanNode, self).__init__()

        self.__parent = None
        self.__children = children

        self.hl_action = hl_action
        self.ll_state = ll_state
        self.hl_state = hl_state
        self.next_ll_state = None
        self.child_generator = None
        self.other_children = []
        self.hlpg_node_ref = None
        self.init_ll_values = None
        self.is_goal = False

    def __hash__(self):
        props = self.hl_state.getAllProps()
        s = ""
        for prop in props:
            s += str(prop)
        sorted_s = ''.join(sorted(s))
        return sorted_s.__hash__()

    def __deepcopy__(self, memodict={}):
        hla_cpy = copy.deepcopy(self.hl_action)
        ll_state_cpy = copy.deepcopy(self.ll_state)
        hl_state_cpy = copy.deepcopy(self.hl_state)
        # ll_plan_copy = copy.deepcopy(self.ll_plan)
        if "type" in memodict.keys():
            children = []
        else:
            children = self.__children[:]


        hlpn = HighLevelPlanNode(hl_action=hla_cpy, ll_state=ll_state_cpy, hl_state=hl_state_cpy,children=children)

        hlpn.next_ll_state = copy.deepcopy(self.ll_state)
        hlpn.init_ll_values = copy.deepcopy(self.init_ll_values)
        return hlpn


    def set_parent(self, parent):
        self.__parent = parent

    def set_children(self, children):
        self.__children = children

    def set_ll_state(self, ll_state):
        self.ll_state = ll_state

    def set_hl_state(self, hl_state):
        self.hl_state = hl_state

    def get_parent(self):
        return self.__parent

    def get_children(self):
        return self.__children

    def add_child(self, node):
        self.__children.append(node)

    def save(self, ll_plan, next_ll_state):
        self.ll_plan = ll_plan #copy.deepcopy(ll_plan) #TODO implement deepcopy
        self.next_ll_state = copy.deepcopy(next_ll_state)

    def get_child(self):
        if self.__children is not None and len(self.__children)>0:
            return self.__children[0]
        else:
            return None

    def reset_child_generator(self):
        self.child_generator = None

    def generate_child(self):
        if not self.child_generator:
            self.child_generator = self.__create_generator()
        child = None
        try:
            child = self.child_generator.next()
        except:
            pass

        return child

    def __create_generator(self):
        for child in self.__children:
            yield child

    def add_other_child(self,child):
        self.other_children.append(child)

    def get_other_children(self):
        return self.other_children




    def remove_child(self,child):
        if child in self.__children:
            self.__children.remove(child)