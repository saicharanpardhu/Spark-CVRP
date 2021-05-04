from random import random
import Timer as timer


def find_branching_set(instance):
    sets, count = [], 0
    while count < 10:
        sets += find_sets_on_coboundary(instance)
        count += 1

    #print("none found ab, searched")
    if sets == []:
        return []

    sets = sorted(sets, key=lambda a_set: (sorted(a_set, key=lambda node: instance.costs[(node, 0)])[0] +
                                           sorted(a_set, key=lambda node: instance.costs[(node, 0)])[
                                               1]) / instance.max_cost + sum(
        instance.demands[node] for node in a_set) / instance.capacity, reverse=True)
    return sets[0]

def find_sets_on_coboundary(instance, lb=2.5, ub=3.5):
    sets = Search_set_manager(lb, ub)
    nodes = sorted(instance.nodes, key=lambda node: instance.costs[node, 0], reverse=True)
    # for node in sorted(nodes,key = lambda node : nodes.index(node)*random()):
    #print("none found, searched")
    for node in sorted(nodes, key=lambda node: random()):
        if node == 0:
            continue
        a_set = [node]
        #print("none found, searched")
        explore_set(instance, a_set, sets)
        #print("none found, searched")
    #print("none found bb, searched")
    return sets.valid_sets


class Search_set_manager():
    def __init__(self, lb, ub):
        self.valid_sets = []
        self.searched_sets = []
        self.ub = ub
        self.lb = lb
        self.timer = timer.timer()

    def is_valid(self, a_set, instance):
        if self.lb <= coboundary(a_set, instance) <= self.ub:
            return True
        return False

    def is_invalid(self, a_set, instance):
        return sum(
            instance.demands[node] for node in a_set) / instance.capacity.value > 4 or a_set in self.searched_sets

    def record_set(self, a_set):
        self.valid_sets.append(a_set)


def explore_set(instance, a_set, sets):
    #print("none found,es searched")
    if len(a_set) > 5 or sets.is_invalid(a_set, instance) or len(sets.valid_sets) > 5 or sets.timer.global_time()>3:
        return
    sets.searched_sets.append(a_set)
    if sets.is_valid(a_set, instance):
        sets.record_set(a_set)
        return  # THIS IS A TEST!
    # print(a_set.demand,a_set.set)
    sorted_nodes = sorted(instance.nodes, key=lambda node: instance.costs[(a_set[-1], node)])
    # sorted_nodes = sorted(instance.nodes, key = lambda node : instance.x[max(a_set[-1],node),min(a_set[-1],node)].value if node!=a_set[-1] else inf)
    for node in sorted_nodes:
        if not (node in a_set) and node != 0:
            a_new_set = a_set.copy()
            a_new_set.append(node)
            explore_set(instance, a_new_set, sets)
    del (a_set)



coboundary = lambda a_set, instance: sum(sum(
    instance.x[max(node, node_out), min(node, node_out)].value for node_out in instance.nodes if
    not (node_out in a_set)) for node in a_set)


