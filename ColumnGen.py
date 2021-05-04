from random import random
from numpy import Infinity
from time import time
import CreateModel as cm
import pyomo.environ as pyo
from numpy import ceil
import networkx as nx
from random import shuffle
from itertools import combinations as comb
from numpy.random import choice

def column_generation(instance, mile_stone=Infinity):
    initial_time, feasible_integer_found, loop_count, unmoving_count, solver_success, failure_count, obj_val_old, obj_val_old_global = round(
        time(), 2), False, 1, 0, True, 0, instance.objective_value, instance.objective_value
    while loop_count < 100 and unmoving_count <= 30 and not (
            feasible_integer_found) and solver_success and instance.objective_value < mile_stone:

        feasible_integer_found, cuts_found,instance2 = column_generation_loop( instance, unmoving_count)
        if cuts_found:
            solver_success = cm.solve(instance)

        print("objective: " + str(round(instance.objective_value, 5)) + ", " + " integer, unmoving count:" + str(
            unmoving_count) + ", reduction: " + str(instance.reduction))

        if instance.objective_value - obj_val_old < 0.000001:
            unmoving_count += 1
            if unmoving_count >= (15 if instance.depth > 0 else 30):
                print("no evolution in column_generation, moving on to branching")
                break
        else:
            unmoving_count = 0
        if instance.objective_value - obj_val_old < -0.1:
            print("we are losing objective function value! useful constraints were dropped")
        if not (cuts_found):
            failure_count += 1
            if failure_count > 3:
                print("no evolution in column_generation, moving on to branching")
                break
        else:
            failure_count = 0
        obj_val_old = instance.objective_value
        loop_count += 1

    print("end of column generation (" + str(round(time() - initial_time, 2)) + "s), gain " + str(
        instance.objective_value - obj_val_old_global) + ", objective: " + str(instance.objective_value))

    return feasible_integer_found, solver_success,instance2


def column_generation_loop( instance, unmoving_count, fixing=False):

    if cm.solution_is_integer(instance):
        cm.integerize_solution(instance)
        if feasible_paths(instance):
            print("!!!!! feasible integer solution found with objective value of " + str(
                round(instance.objective_value, 2)))
            #instance_manager.record_feasible_integer_solution(instance)
            return True, False,instance

    success, count = add_c_cap(instance)
    print("capacity cuts: " + str(count))

    remove_inactive_constraints(instance)


    if not (success):
        print("all heurisitcs have failed")
        return False, False,None

    return False, True,None


def remove_inactive_constraints(instance):

    instance.constraints_inactivity += [0 for i in range(len(instance.c_cap) - len(instance.constraints_inactivity))]
    i = 0
    for c, val in list(instance.dual.items())[
                  instance.n.value + (instance.n.value + 2 * (instance.n.value ** 2)):]:
        if val == 0:
            instance.constraints_inactivity[i] += 1
            if instance.constraints_inactivity[i] > 50:
                c.deactivate()
        else:
            instance.constraints_inactivity[i] = 0
        i += 1


def add_c_cap(instance):
    start = len(instance.c_cap)
    success = False

    candidate_cuts = find_components_random(instance.x, instance.n.value)
    candidate_cuts += complementaries(instance,
                                      candidate_cuts)
    success_temp, components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance, candidate_cuts,
                                                                                      position=" random components")
    success = success or success_temp

    candidate_cuts = find_components(instance.x, instance.n.value)
    candidate_cuts += complementaries(instance,
                                      candidate_cuts)
    success_temp, components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance, candidate_cuts,
                                                                                      position=" normal components")
    success = success or success_temp

    count, success_temp = 0, False
    while not (success_temp) and count < 10:
        count += 1
        candidate_cuts = find_components_correct_implementation_given_demand(instance.x, instance.n.value,
                                                                             instance.demands)
        success_temp, components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance, candidate_cuts,
                                                                                          position=" given demand components correct")
        success = success or success_temp
    count, success_temp = 0, False
    while not (success_temp) and count < 10:
        count += 1
        candidate_cuts = find_components_given_demand(instance.x, instance.n.value, instance.demands)
        success_temp, components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance, candidate_cuts,
                                                                                          position=" given demand components")
        success = success or success_temp

    del (candidate_cuts)

    return success, len(instance.c_cap) - start


precision = 0.00000001


def add_capacity_cuts_from_cuts(instance, cuts, ub="rounded", position=None):
    count = 0
    demand_of_non_connected_to_depot = 0
    passage_of_non_connected_to_depot = 0
    components_single_demand_constrainted = []
    for c in cuts:
        demand = 0
        passage = 0
        for node in c:
            demand += instance.demands[node]
            for i in instance.nodes:
                if not (i in c):
                    passage += instance.x[max(node, i), min(node, i)]
        lb = (ceil(demand / instance.capacity) * 2 if ub == "rounded" else demand * 2 / instance.capacity)
        if pyo.value(passage) < lb - precision:
            instance.c_cap.add(passage >= lb - precision)
            count += 1
            if lb == 2:
                components_single_demand_constrainted.append(c)
        if instance.x[c[-1], 0].value < 0.1:
            demand_of_non_connected_to_depot += demand
            passage_of_non_connected_to_depot += passage
    lb = (ceil(
        demand_of_non_connected_to_depot / instance.capacity) * 2 if ub == "rounded" else demand_of_non_connected_to_depot * 2 / instance.capacity)
    if pyo.value(passage_of_non_connected_to_depot) < lb - precision:
        instance.c_cap.add(passage_of_non_connected_to_depot >= lb - precision)
        count += 1
    return count > 0, components_single_demand_constrainted


def find_maxflow_cuts(instance):
    g = [(i, j, {'capacity': 1, 'weigth': instance.x[max(i, j), min(i, j)]}) for i in range(1, instance.n.value) for j
         in range(1, instance.n.value) if i != j]
    for i in range(1, instance.n.value):
        if instance.x[i, 0].value >= instance.demands[i] * 2 / instance.capacity:
            g.append(
                (i, 0, {'capacity': 2, 'weigth': instance.x[i, 0].value - instance.demands[i] * 2 / instance.capacity}))
        else:
            g.append((instance.n.value + 1, i,
                      {'capacity': 2, 'weigth': -instance.x[i, 0].value + instance.demands[i] * 2 / instance.capacity}))
    val, partition = nx.minimum_cut(nx.DiGraph(g), 0, instance.n.value + 1)
    a = list(partition[0])
    if val < sum(max(0, -instance.x[i, 0].value + instance.demands[i] * 2 / instance.capacity) for i in
                 range(1, instance.n.value)):
        # print("max_flow cuts found")
        a.remove(0)
        return [a]
    return []


def find_components(x, n, thresh=None):
    comps = Components(n)
    first_set = [i for i in range(1, n) if x[i, 0].value >= 1 - 0.1]
    for i in first_set:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component(x, i, comps, k, thresh=thresh)
    for i in comps.r:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component(x, i, comps, k, thresh=thresh)
    cuts = []
    for c in comps.c:
        if len(c) > 1:
            cuts.append(c)
    return cuts


def find_integer_components(x, n):
    comps = Components(n)
    first_set = [i for i in range(1, n) if x[i, 0].value >= 1 - 0.1]
    for i in first_set:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_integer_components(x, i, comps, k)
    for i in comps.r:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_integer_components(x, i, comps, k)
    cuts = []
    for c in comps.c:
        if len(c) > 1:
            cuts.append(c)
    return cuts


def find_components_random(x, n, thresh=0.7):
    comps = Components(n)
    first_set = [i for i in range(1, n) if x[i, 0].value >= 1 - 0.1]
    for i in first_set:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component_random(x, i, comps, k, thresh=thresh)
    for i in comps.r:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component_random(x, i, comps, k, thresh=thresh)
    cuts = []
    for c in comps.c:
        if len(c) > 1:
            cuts.append(c)
    return cuts


def find_components_given_demand(x, n, demands):
    comps = Components(n)
    first_set = [i for i in range(1, n) if x[i, 0].value >= 1 - 0.1]
    for i in first_set:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component_given_demand(x, i, comps, k, demands)
    for i in comps.r:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component_given_demand(x, i, comps, k, demands)
    cuts = []
    for c in comps.c:
        if len(c) > 1:
            cuts.append(c)
    return cuts


def find_components_correct_implementation_given_demand(x, n, demands):
    comps = Components(n)
    first_set = [i for i in range(1, n) if x[i, 0].value >= 1 - 0.1]
    for i in first_set:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component_correct_implementation_given_demand(x, i, comps, k, demands)
    for i in comps.r:
        if comps.free(i):
            comps.c.append([])
            k = len(comps.c) - 1
            explore_component_correct_implementation_given_demand(x, i, comps, k, demands)
    cuts = []
    for c in comps.c:
        if len(c) > 1:
            cuts.append(c)
    return cuts


def find_random_cuts(x, n):
    amount = 5
    cuts = [[] * amount]
    for i in range(len(cuts)):
        for j in range(max(1, int(random() * n))):
            k = max(1, int(random() * n))
            if not (k in cuts[i]):
                cuts[i].append(k)
    return cuts


def feasible_paths(instance):
    roads = find_integer_components(instance.x, instance.n.value)
    success, components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance, roads)
    return not (success)


def complementaries(instance, cuts):
    return [complementary(instance, c) for c in cuts if complementary(instance, c) != []]


def complementary(instance, cut):
    return [i for i in range(1, instance.n.value) if not (i in cut)]


vals_for_projection = [0, 1]


def closest(val, list=vals_for_projection):
    l = sorted(list, key=lambda x: abs(x - val))
    return l[0]


'''	 CONNETED COMPONENTS CUTS SUPPORT '''


class Components:
    def __init__(self, n):
        self.r = [i for i in range(1, n)]
        shuffle(self.r)
        self.c = []

    def free(self, i):
        return i in self.r

    def take(self, i, k):
        self.r.remove(i)
        self.c[k].append(i)


def explore_component_random(x, i, comps, k, thresh=0.7):
    comps.take(i, k)
    if connected(x, i, 0) and random() > thresh:
        return
    for j in comps.r:
        if connected(x, i, j):
            explore_component_random(x, j, comps, k)


def explore_component_given_demand(x, i, comps, k, demands):
    comps.take(i, k)
    if sum(demands[i] for i in comps.c[k]) >= 8 and connected(x, i, 0):
        return
    for j in comps.r:
        if connected(x, i, j):
            explore_component_given_demand(x, j, comps, k, demands)


def explore_component(x, i, comps, k, thresh=None):
    comps.take(i, k)
    for j in comps.r:
        if connected(x, i, j, thresh=thresh):
            explore_component(x, j, comps, k)


def explore_component_correct_implementation(x, i, comps, k):
    comps.take(i, k)
    for j in comps.r:
        if connected(x, i, j):
            explore_component_correct_implementation(x, j, comps, k)
            break


def explore_component_correct_implementation_given_demand(x, i, comps, k, demands):
    # actually does what I had thought it did : i want a true road at the end, one with no junctions!
    comps.take(i, k)
    if sum(demands[i] for i in comps.c[k]) >= 8 and connected(x, i, 0):
        return
    for j in comps.r:
        if connected(x, i, j):
            explore_component_correct_implementation(x, j, comps, k)
            break


def explore_integer_components(x, i, comps, k):
    comps.take(i, k)
    for j in comps.r:
        if connected_specific(x, i, j):
            explore_integer_components(x, j, comps, k)


def connected(x, i, j, thresh=None):
    a, b = max(i, j), min(i, j)
    return x[a, b].value > (0.9 if thresh == None else thresh)


def connected_specific(x, i, j):
    a, b = max(i, j), min(i, j)
    return x[a, b].value > 1 - 0.1





def shrink_graph(instance, components_single_demand_constrainted):

    shrunk_instance = instance.clone()
    start = time()
    while time() - start < 4:


        for c in components_single_demand_constrainted:
            c = c.sort(
                reverse=True)
            if verify_safe_shrinking(shrunk_instance, c):
                shrink_set(shrunk_instance, c)

        i = choice(shrunk_instance.nodes)
        j = choice(list(shrunk_instance.nodes)[:i])
        if shrunk_instance.x[i, j].value >= 1:
            shrink_set(shrunk_instance, [(i, j)])
    return shrunk_instance


def outgoing_passage(instance, set):
    total = 2 * len(set)
    for i, j in comb(set, 2):
        total -= instance.x[set[i], set[j]].value
    return total


def verify_safe_shrinking(instance, set):
    if len(set) > 4:
        return False
    if outgoing_passage(instance, set) > 2:
        return False
    for i in range(2, len(set)):
        sub_sets = comb(set, i)
        for sub in sub_sets:
            if outgoing_passage(instance, sub) < 2:
                return False
    return True


def shrink_set(instance, set):
    demand = outgoing_passage(instance, set)
    super_node = set[0]
    instance.demands[super_node] = demand
    for node in instance.nodes:
        if node not in set:
            instance.costs[max(node, super_node), min(node, super_node)] = sum(
                instance.costs[max(node, i), min(node, i)] for i in set)
    for node in set[1:]:
        for node2 in instance.nodes:
            instance.x[max(node, node2), min(node, node2)].value = 0
            instance.x[max(node, node2), min(node, node2)].fixed = True
