import sys
import pyomo.environ as pyo
from random import random
from pyomo.opt import SolverFactory, TerminationCondition
from numpy import sqrt

#opt = SolverFactory("gurobi", solver_io="python")
opt = SolverFactory('gurobi')
opt.options['threads'] = 8


def createModel(file):
    model = initialize_model()
    instance = construct_instance(model, file)
    instance, locations = initialize_instance(instance)
    return instance, locations


def initialize_model(locations=True):
    print("initialising model")

    model = pyo.AbstractModel("CVRP")

    model.number_of_vehicles = pyo.Param(within=pyo.PositiveIntegers)
    model.n = pyo.Param(within=pyo.PositiveIntegers)
    model.capacity = pyo.Param(within=pyo.PositiveIntegers)
    model.entry_type = pyo.Param(within=pyo.Any)
    model.nodes = pyo.RangeSet(0, model.n - 1)
    model.demands = pyo.Param(model.nodes)
    model.locations = pyo.Param(model.nodes, pyo.RangeSet(0, 1))
    model.costs = pyo.Param(model.nodes, model.nodes)
    return model


def construct_instance(model, file):
    if ".dat" in file:
        try:
            instance = model.create_instance(file)
        except:
            raise Exception("file not found")
    elif ".vrp" in file:
        try:
            file_name = file[:-4] + ".dat"
            create_dat(file, file_name)
            instance = model.create_instance(file_name)
        except:
            raise Exception("vrp not found or could not be translated correctly")
    else:
        raise NameError("given file name could not be resolved")
    instance.file = file
    return instance


def initialize_instance(instance):
    if instance.entry_type.value == "COORD":

        locations = to_list_locations(instance)

    else:

        locations = [(random() * 20, random() * 20) for i in range(instance.n.value)]

    max_cost = 0
    for k in instance.costs.values():
        if k > max_cost:
            max_cost = k
    instance.max_cost = max_cost


    instance.x = pyo.Var(instance.nodes, instance.nodes, bounds=set_bounds)

    for i in instance.nodes:
        for j in instance.nodes:
            if i <= j:
                del (instance.x[i, j])
            # instance.x[i,j].deactivate()


    instance.flow = pyo.Var(instance.nodes, instance.nodes, bounds=set_bounds_flow)


    instance.objective = pyo.Objective(
        expr=sum(instance.costs[i, j] * instance.x[i, j] for i in instance.nodes for j in instance.nodes if i > j))


    instance.c_deg = pyo.Constraint(instance.nodes, rule=rule_deg)



    instance.c_flow = pyo.Constraint(instance.nodes, rule=rule_flow)
    instance.c_flow_deg = pyo.Constraint(instance.nodes, instance.nodes, rule=rule_flow_deg)
    instance.c_flow_deg_indirect = pyo.Constraint(instance.nodes, instance.nodes, rule=rule_flow_deg_indirect)


    instance.c_cap = pyo.ConstraintList()

    instance.dual = pyo.Suffix(direction=pyo.Suffix.IMPORT_EXPORT)


    instance.c_branch = pyo.ConstraintList()
    instance.branching_sets = []
    instance.branched_indexes = []

    instance.reduction = 1

    instance.depth = 0
    instance.constraints_inactivity = []
    instance.branching_indexes = []

    reduce_problem(instance,0.6)

    fix_edges_to_depot(instance)


    opt.solve(instance)
    instance.objective_value = pyo.value(instance.objective)

    print("finished constructing instance model")
    return instance, locations


def fix_edges_to_depot(instance):
    for i in range(1, instance.n.value):
        instance.x[i, 0].domain = pyo.NonNegativeIntegers


def reduce_problem(instance,thresh):
    for i, j in [(a, b) for a in range(instance.n.value) for b in range(a)]:
        if not ((i, j) in instance.branching_indexes):
            if instance.costs[(i, j)] > thresh * instance.max_cost:
                instance.x[(i, j)].value = 0
                instance.x[(i, j)].fixed = True
            else:
                instance.x[(i, j)].fixed = False
    instance.reduction = thresh


def set_bounds(instance, i, j):

    if (i <= j):
        return (-1, -1)
    if (j == 0):
        return (0, 2)

    return (0, 1)


def set_bounds_flow(instance, i, j):
    return (0, 1)


def rule_deg(instance, i):

    return sum((instance.x[i, j] if i > j else instance.x[j, i]) for j in instance.nodes if i != j) == (
        2 if i > 0 else 2 * instance.number_of_vehicles)


def rule_flow(instance, i):
    return pyo.Constraint.Skip if i == 0 else (sum(instance.flow[i, j] for j in instance.nodes if i != j) >= sum(
        instance.flow[j, i] for j in instance.nodes if i != j) + instance.demands[i] / instance.capacity)


def rule_flow_deg(instance, i, j):
    return pyo.Constraint.Skip if j >= i else (instance.x[i, j] >= instance.flow[i, j])


def rule_flow_deg_indirect(instance, i, j):
    return pyo.Constraint.Skip if j >= i else (instance.x[i, j] >= instance.flow[j, i])


def to_list_locations(instance):
    d = []
    for i in instance.nodes:
        d.append((instance.locations[i, 0], instance.locations[i, 1]))
    return d


def sanitize(string, seperator=""):
    chars = ['\t', '\n', "_", "-", ":"]
    for c in chars:
        while c in string:
            i = string.index(c)
            try:
                string = string[:i] + seperator + string[i + len(c):]
            except IndexError:
                print("sanitize out of range with : ")
                print(string + "c = " + c + " i = " + str(i) + "len = " + str(len(c)))

    offset = 0
    for j in range(1, len(string)):
        if string[j - offset] == string[j - 1 - offset] == seperator:
            string = string[:j - 1] + string[j:]
            offset += 1
    return string


def create_dat(in_name, out_name):
    with open(in_name, "r") as f:

        lines = f.readlines()

        for i in range(len(lines)):
            line = sanitize(lines[i], " ")
            line = line.split(" ")
            while "" in line:
                line.remove("")
            lines[i] = line

        found = False
        for line in lines:
            if line[0] == "DIMENSION":
                found = True
                try:
                    dimension = int(float(line[-1]))
                except:
                    raise NameError(line)
                break
        if not (found):
            raise Exception("Could not locate DIMENSION parameter")

        # capacity
        found = False
        for line in lines:
            if line[0] == "CAPACITY":
                found = True
                try:
                    capacity = int(float(line[-1]))
                except:
                    raise NameError(line)
                break
        if not (found):
            raise Exception("Could not locate DIMENSION parameter")

        # demands
        found = False
        start_index_demand = 0
        for i in range(len(lines)):
            line = lines[i]
            if line[0] == "DEMAND" and line[1] == "SECTION":
                start_index_demand = i + 1
                found = True
                break
        if not (found):
            raise Exception("Could not locate DEMAND SECTION parameter")


        depot_set_apart = int(float(lines[start_index_demand][1])) > 0
        indexing_offset = int(float(lines[start_index_demand][0])) - (1 if depot_set_apart else 0)

        demands = [0]
        for i in range(0 if depot_set_apart else 1, dimension):
            line = lines[start_index_demand + i]
            try:
                demands.append(int(float(sanitize(line[1]))))
            except ValueError:
                print("error while adding demands with : ")
                print(start_index_demand + i)
                print(line)
                print(line[1])
                print(float(sanitize(line[1])))
            except IndexError:
                print("error while adding demands with : ")
                print(start_index_demand + i)
                print(line)
                print(line[1])
                print(float(sanitize(line[1])))

        # locations/weights
        found = False
        start_index = 0
        for i in range(len(lines)):
            line = lines[i]
            if line[0] == "NODE" and line[1] == "COORD" and line[2] == "SECTION":
                found = True
                start_index = i + 1
                break
        if found:
            input_type = "COORD"
            locations = {}
            line = lines[start_index]
            for i in range(dimension):
                line = lines[start_index + i]
                try:
                    n, i, j = int(float(sanitize(line[0]))) - indexing_offset, float(sanitize(line[1])), float(
                        sanitize(line[2]))
                except IndexError:
                    print("mauvaise indentation dans locations avec : ")
                    print(line)
                    print(n)
                    print(i)
                    print(j)
                    print(len(line))
                locations[(n, 0)] = i
                locations[(n, 1)] = j
            if depot_set_apart:
                depot_found = False
                depot_index = 0
                for i in range(len(lines)):
                    line = lines[i]
                    if line[0] == "DEPOT" and line[1] == "SECTION":
                        depot_index = i + 1
                        break
                if not (depot_found):
                    raise Exception("could not locate DEPOT SECTION paremeter")
                line = lines[depot_index]
                locations[(0, 0)] = float(line[0])
                locations[(0, 1)] = float(line[1])
        else:
            raise Exception("WEIGHT INPUT not yet implemented, must take into account the different types of wieghts input (lower,higher,complete etc.)")
            for i in range(len(lines)):
                line = lines[i]
                if line[0] == "EDGE" and line[1] == "WEIGHT" and line[2] == "SECTION":
                    start_index = i + 1
                    found = True
                    break
            if not (found):
                raise Exception("Could locate neither EDGE WEIGTH SECTION nor NODE COORD SECTION parameters")
            input_type = "WEIGHT"
            weights = {}
            for i in range(dimension):
                line = lines[start_index + i]
                for j in range(len(line)):
                    try:
                        weights[(i, j)] = float(line[j])
                    except:
                        raise Exception(
                            "Error in writing of weights with paremeters " + str(i) + " " + str(j) + " " + str(
                                start_index))


        found = False
        for i in range(len(lines)):
            line = lines[i]
            if line[0] == "VEHICLES":
                found = True
                number_of_vehicles = int(float(line[-1]))
                break
        if not (found):
            for i in range(len(lines)):
                line = lines[i]
                if line[0] == "NAME" and line[-1][0] == "k":
                    found = True
                    number_of_vehicles = int(float(line[-1][1:]))
                    break
        if not (found):
            number_of_vehicles = int(sum(demand for demand in demands) / capacity) + 4

    weights = {}
    if input_type == "COORD":
        for i in range(dimension):
            for j in range(dimension):
                weights[i, j] = sqrt(
                    (locations[(i, 0)] - locations[(j, 0)]) ** 2 + (locations[(i, 1)] - locations[(j, 1)]) ** 2)



    with open(out_name, "w") as f:

        # number of vehicles
        f.write("param number_of_vehicles := " + str(number_of_vehicles) + ";\n")
        f.write("\n")

        # dimension
        f.write("param n := " + str(dimension) + ";\n")
        f.write("\n")

        # capacity
        f.write("param capacity := " + str(capacity) + ";\n")
        f.write("\n")

        # type of entry
        f.write("param entry_type := " + input_type + ";\n")
        f.write("\n")

        if input_type == "COORD":
            f.write("param locations := \n")
            for k in locations.keys():
                f.write(str(k[0]) + " " + str(k[1]) + " " + str(locations[k]) + "\n")
            f.write(";\n")
            f.write("\n")

            f.write("param costs := \n")
            for k in weights.keys():
                f.write(str(k[0]) + " " + str(k[1]) + " " + str(weights[k]) + "\n")
            f.write(";\n")
            f.write("\n")

        elif input_type == "WEIGHT":
            f.write("param locations := \n")
            for i in range(dimension):
                f.write(str(i) + " 0 -1" + "\n")
                f.write(str(i) + " 1 -1" + "\n")
            f.write(";\n")
            f.write("\n")

            f.write("param costs := \n")
            for k in weights.keys():
                f.write(str(k[0]) + " " + str(k[1]) + " " + str(weights[k]) + "\n")
            f.write(";\n")
            f.write("\n")

        else:
            raise Exception("We screwed up : input_type not recognized at time of writing")

        # demands
        f.write("param demands := \n")
        for i in range(dimension):
            f.write(str(i) + " " + str(demands[i]) + "\n")
        f.write(";\n")


def solve(instance):
    res = opt.solve(instance)
    if res.solver.termination_condition == TerminationCondition.infeasible:
        if instance.reduction >= 1:
            return False
        else:
            reduce_problem(instance, min(instance.reduction + 0.2, 1))
            solve(instance)
    instance.objective_value = pyo.value(instance.objective)
    return True


def solution_is_integer(instance):

    for b in [(i, j) for i in range(instance.n.value) for j in range(i)]:
        val = instance.x[b].value
        expr = abs(round(val) - val) > 0.1
        if expr:
            return False
    return True


def integerize_solution(instance):

    for b in instance.x.keys():
        instance.x[b].value = round(instance.x[b].value)
