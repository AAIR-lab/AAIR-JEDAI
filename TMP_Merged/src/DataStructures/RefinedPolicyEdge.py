class RefinedPolicyEdge(object):
    def __init__(self,ll_plan,generated_values,has_mp,exec_seq,effect,ll_values = None):
        self.ll_plan = ll_plan
        self.generated_values = generated_values
        self.has_mp = has_mp
        self.exec_seq = exec_seq
        self.effect = effect
        self.ll_values = ll_values