from queue import PriorityQueue
from functools import total_ordering
import copy

GAMMA = 0.01


class Node:
    def __init__(self):
        self.current_state = set()
        self.problem = None
        self.plan = []

    def get_state(self):
        return self.current_state

    def goal_test(self):
        pass

    def get_successors(self):
        pass

    def get_plan(self):
        return self.plan

    def get_heuristic(self):
        return 0

    def get_fvalue(self):
        return self.get_heuristic() + len(self.get_plan())

    def expand_operations(self):
        pass


@total_ordering
class AbsNode(Node):
    def __init__(self, problem, current_state, curr_plan=[], current_foils=set(), cogn_bound=-1):
        self.current_state = set(current_state)
        self.current_foils = current_foils
        self.problem = problem
        self.plan = curr_plan
        self.cognitive_bound = cogn_bound
        self.current_cost = 0
        self.current_chunk = 0

    def goal_test(self):
        # TODO use cognitive bound?
        if len(self.current_foils) == 0:  # and self.current_cost >= int(cb):
            return True

    def get_successors(self, cb=-1):
        possible_concret = set()
        successor_list = []
        for model in self.current_state:
            try:
                edge_list = self.problem.inverse_edges[model]
                for e in edge_list.keys():
                    possible_concret.add(e)
            except:
                pass
        for prop in possible_concret:
            tmp_succ = set()
            for model in self.current_state:
                try:
                    tmp_succ.add(self.problem.inverse_edges[model][prop])
                except:
                    tmp_succ.add(model)
            tmp_model = AbsNode(self.problem, tmp_succ, self.plan + [prop], self.current_foils)
            tmp_model.current_cost = self.current_cost + self.problem.concret_costs[prop]
            # tmp_set = set()
            # for m2 in tmp_model.current_state:
            #    tmp_set |= set(tmp_model.problem.find_unresolved_foils(m2,tmp_model.current_foils))
            # tmp_model.current_foils = copy.deepcopy(tmp_set)
            successor_list.append(tmp_model)
        return successor_list

    def get_fvalue(self):
        return self.get_heuristic() + self.current_cost

    def get_chunk_fvalue(self):
        return len(self.current_foils)

    def __eq__(self, other):
        return (list(self.current_state)[0]) == (list(other.current_state)[0])

    def __lt__(self, other):
        return (list(self.current_state)[0]) < (list(other.current_state)[0])

    def expand_operations(self):
        foils = set()
        for m in self.current_state:
            foils |= set(self.problem.find_unresolved_foils(m, self.current_foils))
        self.current_foils = copy.deepcopy(foils)


@total_ordering
class LatNode(Node):
    def __init__(self, lattice, current_state, curr_plan=[]):
        self.current_state = current_state
        self.problem = lattice
        self.plan = curr_plan

    def goal_test(self):
        return self.problem.verify_the_lattice(self.current_state)

    def get_successors(self):
        return self.problem.get_all_successor_nodes(self)

    def get_state(self):
        return str(self.current_state["edges"])

    def __eq__(self, other):
        return (list(self.current_state)[0]) == (list(other.current_state)[0])

    def __lt__(self, other):
        return (list(self.current_state)[0]) < (list(other.current_state)[0])


def greedy_search(start_state):
    fringe = PriorityQueue()
    closed = set()
    nodes_expanded = 0
    fringe.put((0, start_state))

    while not fringe.empty():
        val, node = fringe.get()
        node.expand_operations()
        if node.goal_test():
            print("Goal Found! Number of Nodes Expanded =", nodes_expanded)
            return node  # .get_plan()
        if node.get_state() not in closed:
            closed.add(frozenset(node.get_state()))
            successor_list = node.get_successors()
            nodes_expanded += 1
            print("Number of Nodes Expanded =", nodes_expanded)

            while successor_list:
                candidate_node = successor_list.pop()
                fringe.put((candidate_node.get_fvalue(), candidate_node))

    return None
