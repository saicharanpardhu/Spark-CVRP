from pyspark import SparkContext, SparkConf
from six.moves import xrange
from pyspark import SparkContext
from pyspark.streaming import StreamingContext
import CreateModel as cm
import Timer as timer
from pyomo.core import value
from OrToolsObjects import *
from BranchGen import *
from ColumnGen import *
import time

if __name__ == '__main__':
    conf = SparkConf().setAppName("cvrp-pardhu")
    sc = SparkContext(conf=conf)
    tim = timer.timer()

    instance, locations = cm.createModel("C:\\cvrp\\P-n16-k8.vrp")
    queue = []
    length = 0
    best_feasible_integer_solution = None
    branches_cut = 0
    partial_solution_recorded = []
    total_length = 0
    upper_bound = 0


    def record_feasible_integer_solution(instance):
        global upper_bound, best_feasible_integer_solution
        print("no optimal integer solution found")
        if instance.objective_value < upper_bound:
            upper_bound = instance.objective_value
            for b in instance.x.keys():
                instance.x[b].value = round(instance.x[b].value)
            best_feasible_integer_solution = instance.objective


    def empty():
        return


    def upper_bound(instance, distances):
        data = DataProblemDistances(instance, distances)
        distance_evaluator = CreateDistanceEvaluator2(data).distance_evaluator
        model_parameters = pywrapcp.DefaultRoutingModelParameters()

        manager = pywrapcp.RoutingIndexManager(data.num_locations, data.num_vehicles, data.depot)
        routing = pywrapcp.RoutingModel(manager, model_parameters)

        transit_callback_index = routing.RegisterTransitCallback(distance_evaluator)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
        add_capacity_constraints(routing, data, demand_evaluator)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        search_parameters.time_limit.seconds = 3
        search_parameters.local_search_operators.use_path_lns = pywrapcp.BOOL_FALSE
        search_parameters.local_search_operators.use_inactive_lns = pywrapcp.BOOL_FALSE

        search_parameters.local_search_operators.use_tsp_opt = pywrapcp.BOOL_FALSE
        search_parameters.use_full_propagation = pywrapcp.BOOL_TRUE

        assignment = routing.SolveWithParameters(search_parameters)
        t = type(empty())
        count = 0
        while type(assignment) == t:
            data._num_vehicles += 1
            count += 1
            manager = pywrapcp.RoutingIndexManager(data.num_locations, data.num_vehicles, data.depot)
            routing = pywrapcp.RoutingModel(manager, model_parameters)
            transit_callback_index = routing.RegisterTransitCallback(distance_evaluator)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
            add_capacity_constraints(routing, data, demand_evaluator)
            assignment = routing.SolveWithParameters(search_parameters)
        return calculate_distance(data, manager, routing, assignment, distance_evaluator)


    def calculate_distance(data, manager, routing, assignment, distance_evaluator):
        total_dist = 0
        for vehicle_id in xrange(data.num_vehicles):
            index = routing.Start(vehicle_id)
            route_dist = 0
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                next_node_index = manager.IndexToNode(assignment.Value(routing.NextVar(index)))
                route_dist += distance_evaluator(node_index, next_node_index)
                index = assignment.Value(routing.NextVar(index))
            node_index = manager.IndexToNode(index)
            total_dist += route_dist
        return total_dist


    def add_capacity_constraints(routing, data, demand_evaluator):
        capacity = "Capacity"
        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_evaluator)
        routing.AddDimension(
            demand_callback_index,
            0,
            data.vehicle.capacity,
            True,
            capacity)


    def branch(instance):
        branch_set = find_branching_set(instance)
        print("branching constraint found over set " + str(branch_set) + " with demand " + str(
            round(sum(instance.demands[i] / instance.capacity.value for i in branch_set),
                  4)) + " and coboundary " + str(
            sum(sum(instance.x[max(node, node_out), min(node, node_out)].value for node_out in instance.nodes if
                    not (node_out in branch_set)) for node in branch_set)))
        if branch_set == []:
            index = find_branching_index(instance)
            if index == (-1, -1):
                return None, None
            instance.branching_indexes.append(index)

            # creating new instances ! recycling the old one into one of the new branches
            instance.x[index].fixed = True
            instance.branched_indexes.append(index)
            instance2 = instance.clone()
            instance.x[index].value = 0
            instance2.x[index].value = 1

            depth = instance.depth
            instance.depth = depth + 1
            instance2.depth = depth + 1
            instances = [instance, instance2]
            depth = instances[0].depth
            for inst in instances:
                inst.depth = depth + 1

            if instance == None:
                return []
            print("ADDING INSTANCES of " + str(len(instances)))
            for instanceq in instances:
                solver_success2 = cm.solve(instanceq)
                if solver_success2:
                    instanceq.lower_bound = instanceq.objective_value
            return instances

        instance.branching_sets.append(branch_set)
        instance2 = instance.clone()
        instance3 = instance.clone()
        expression = sum(
            sum(instance.x[max(i, j), min(i, j)] for j in instance.nodes if not (j in branch_set)) for i in branch_set)
        instance.c_branch.add(expression == 2)
        expression = sum(
            sum(instance2.x[max(i, j), min(i, j)] for j in instance2.nodes if not (j in branch_set)) for i in
            branch_set)
        instance2.c_branch.add(expression == 4)
        expression = sum(
            sum(instance3.x[max(i, j), min(i, j)] for j in instance3.nodes if not (j in branch_set)) for i in
            branch_set)
        instance3.c_branch.add(expression >= 6)

        instances = [instance, instance2, instance3]

        depth = instances[0].depth
        for inst in instances:
            inst.depth = depth + 1

        print("finished creating new instances")

        if instance == None:
            return []
        print("ADDING INSTANCES of " + str(len(instances)))
        for instance in instances:
            solver_success = cm.solve(instance)
            if solver_success:
                instance.lower_bound = instance.objective_value
        return instances


    def find_branching_index(instance):
        bridges = [(i, j) for i in range(instance.n.value) for j in range(i)]
        indexes = sorted(bridges, key=lambda bridge: min(abs(instance.x[bridge].value - 0.5),
                                                         abs(instance.x[bridge].value - 0.75)))[
                  :min(len(bridges), 5)]
        best_boost = 0
        for ind in indexes:
            instance.x[ind].value = 1
            instance.x[ind].fixed = True
            cm.opt.solve(instance)
            a = value(instance.objective)
            instance.x[ind].value = 0
            cm.opt.solve(instance)
            b = value(instance.objective)
            c = min(a, b)
            if c > best_boost:
                best_boost = c
                index = ind
            instance.x[ind].fixed = False
        return index


    def generateColumns(instance):
        print("starting processing of new instance with lower_bound of " + str(instance.lower_bound))

        feasible_integer_found, solver_success, inst = column_generation(instance)
        if inst is not None:
            return inst
        if not feasible_integer_found and solver_success:
            return branch(instance)
        else:
            return None


    def createRDDandSubmit(intances):
        if intances is not None and len(intances) > 0:
            rdd = sc.parallelize(intances, len(intances))
            mappedStream = rdd.map(lambda x: generateColumns(x))
            outInst = mappedStream.collect()
            newInstances = []
            for x in outInst:
                if x is not None and type(x) is not list:
                    record_feasible_integer_solution(x)

            for x in outInst:
                if x is not None and type(x) is list:
                    for y in x:
                        if y.lower_bound <= upper_bound and y.x[1, 0].value is not None:
                            newInstances.append(y)
            res = createRDDandSubmit(newInstances)

        return True


    upper_bound = upper_bound(instance, instance.costs)
    instance.lower_bound = instance.objective_value
    insts = [instance]
    createRDDandSubmit(insts)

    if best_feasible_integer_solution is None:
        print("no optimal integer solution found")
    else:
        print("best feaible integer solution found has objective value of " + str(
            pyo.value(best_feasible_integer_solution.value)))
    print(tim.global_time(), " total running time")
