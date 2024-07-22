class StochasticPolicy:
    @staticmethod
    def select_child_using_props(policy_tree, node, choice, children):
        # import IPython
        # IPython.embed()
        for child in children:
            edge = policy_tree.get_edge(node, child)
            placed_plank = edge.generated_values["plank"]
            props = child.hl_state.getTrueProps()
            for prop in props:
                if "human_placed" in prop and placed_plank in prop:
                    fin_prop = prop
                    break

            if "location1" in fin_prop and choice == 0:
                selected_child = child
                break
            elif "location2" in fin_prop and choice == 1:
                selected_child = child
                break
                    # print("Child "+str(choice)+" selected")
        return selected_child