import random

class StochasticPolicy:
    @staticmethod
    def select_child_using_props(policy_tree, node, choice, children):
        # import IPython
        # IPython.embed()
        
        assert len(children) <= 2
        return children[choice]
    