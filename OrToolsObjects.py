from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

dilate = lambda d : d

class DataProblemDistances():
    """Stores the data for the problem"""

    def __init__(self, instance, distances):
        self._vehicle = Vehicle(instance.capacity.value)
        self._num_vehicles = instance.number_of_vehicles.value
        self._distances = [[0] * instance.n.value] * instance.n.value
        for i in range(instance.n.value):
            for j in range(instance.n.value):
                self._distances[i][j] = distances[i, j]
        self._depot = 0
        self._demands = instance.demands

    @property
    def vehicle(self):
        """Gets a vehicle"""
        return self._vehicle

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self._demands)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot

    @property
    def demands(self):
        """Gets demands at each location"""
        return self._demands


class Vehicle():
    """Stores the property of a vehicle"""

    def __init__(self, cap):
        """Initializes the vehicle properties"""
        self._capacity = cap

    @property
    def capacity(self):
        """Gets vehicle capacity"""
        return self._capacity


class CreateDistanceEvaluator2(object):
    """Creates callback to return distance between points."""

    def __init__(self, data):
        """Initializes the distance matrix."""
        self._distances = data._distances

    def distance_evaluator(self, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        global dilate
        return dilate(self._distances[from_node][to_node])


class CreateDemandEvaluator(object):  # pylint: disable=too-few-public-methods
    """Creates callback to get demands at each location."""

    def __init__(self, data):
        """Initializes the demand array."""
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return self._demands[from_node]

