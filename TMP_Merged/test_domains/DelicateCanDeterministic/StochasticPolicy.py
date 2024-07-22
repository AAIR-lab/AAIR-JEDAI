class StochasticPolicy:
    @staticmethod
    def select_child_using_props(policy_tree, node, choice, children):
        for child in children:
            edge = policy_tree.get_edge(node, child)
            obj = edge.generated_values["obj"]
            props = child.hl_state.getTrueProps()
            for prop in props:
                if "curshed" in prop and obj in prop:
                    fin_prop = prop
                    break
            if "curshed" in fin_prop and choice == 0:
                selected_child = children[0]
                break
            elif "curshed" not in fin_prop and choice == 1:
                selected_child = children[1]
                break
        return selected_child