import copy

class Edge(object):
    def __init__(self,parent=None,child=None,hl_action=None,ll_action_spec= None,ll_plan=None,prob=0,debug_name = ""):
        self.parent = parent
        self.child = child
        self.hl_action =hl_action
        self.ll_action_spec = ll_action_spec
        self.ll_plan = ll_plan
        self.prob = prob
        self.debug_name = debug_name
        self.refined_ll_values = None
        self.generated_values = None
        self.has_mp = False
        self.refined_ll_state = None


    def __deepcopy__(self, memodict={}):
        hl_action_cpy = copy.deepcopy(self.hl_action)
        ll_action_spec_copy = copy.deepcopy(self.ll_action_spec)
        prob_copy = copy.deepcopy(self.prob)
        ll_plan_copy = copy.deepcopy(self.ll_plan)
        debug_name_copy = copy.deepcopy(self.debug_name)
        refined_ll_state_copy = copy.deepcopy(self.refined_ll_state)
        edge_copy = Edge(hl_action=hl_action_cpy,ll_action_spec=ll_action_spec_copy,
                         prob=prob_copy,ll_plan=ll_plan_copy,debug_name=debug_name_copy)
        refined_ll_values_copy = copy.deepcopy(self.refined_ll_values)
        generated_values_copy = copy.deepcopy(self.generated_values)
        has_mp_copy = copy.deepcopy(self.has_mp)
        edge_copy.refined_ll_values = refined_ll_values_copy
        edge_copy.generated_values = generated_values_copy
        edge_copy.has_mp = has_mp_copy
        edge_copy.refined_ll_state = refined_ll_state_copy
        return edge_copy

    def add_parent(self,parent):
        self.parent = parent

    def add_child(self,child):
        self.child = child

    def get_parent(self):
        return self.parent

    def get_child(self):
        return self.child