# Spark-CVRP
Vehicle Routing Problem or simply VRP is a well known combinatorial optimization problem and a generalization of the travelling salesman problem. A definition of the problem is this: We have a number of customers that have a demand for a delivery. Which are the optimal (minimal) routes for a fleet of vehicles starting from a single point (depot) to deliver the requested goods in all customers. Finding optimal solution is a NP-hard problem so heuristic strategies are proposed for approximation of the optimal solution. For more about the problem see: https://en.wikipedia.org/wiki/Vehicle_routing_problem.

The variant of VRP that we are solving here is Capacitated Vehicle Routing Problem (CVRP). It can formally be defined as follows:

Let G = (V, A) be a graph where V is the vertex set and A is the arc set. One of the vertices represents the depot at which a fleet of m identical vehicles of capacity Q is based, and the other vertices customers that need to be serviced. With each customer vertex vi are associated a demand qi. With each arc (vi, vj) of A are associated a cost cij. The CVRP consists in finding a set of routes such that:

Each route begins and ends at the depot;
Each customer is visited exactly once by exactly one route;
The total demand of the customers assigned to each route does not exceed Q;
The total cost of the routes is minimized.
