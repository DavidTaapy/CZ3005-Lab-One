# Import required libraries and classes
from math import sqrt
from queue import PriorityQueue
from Classes.Data import result_to_txt

# A-Star Search Algorithm
def search(Data, has_energy_constraint, weight = 1):  # Default weight of 1 if none is specified
    # Number of paths that did not satisfy energy constraint
    fail_count = 0
    # File name based on whether there is energy constraint
    if has_energy_constraint:
        file_name = "Task3_AStar_with_energy_constraint"
    else:
        file_name = "Task3_AStar_without_energy_constraint"
    # Start A-Star Search
    print("Initialising A-Star Search...")
    # List of paths aka frontier - priority given to lowest f(n)
    paths = PriorityQueue()
    # Get initial f(n), which is f(n) = 0 + h(n) as distance traversed is still 0
    initial_distance = get_heuristic(Data, Data.start_node, weight)
    # Structure of Queue (Distance, Cost, Node) so paths will be automatically sorted according to the shortest distance
    paths.put((initial_distance, 0, Data.start_node))
    # Empty list to initialise list of nodes that have been explored
    explored = []
    # Dictionary to store costs of visited nodes
    cost_dict = {Data.start_node: 0}
    # Store current distances for explored paths
    distance_so_far = {Data.start_node: 0}
    # Start the search
    print("Starting search...")
     # As long as priority queue has vertices
    while paths:
        # Get the first path in priority queue (Shortest distance traversed)
        curr_distance, curr_cost, curr_path = paths.get()
        # Get the last node added to the path
        curr_node = curr_path[-1]
        # Remove heuristic value of current node so it can be updated with neighbour's heuristic value
        heuristic_value = get_heuristic(Data, curr_node, weight) # Straight-line distance from current node to goal node
        curr_distance -= heuristic_value
        # Mark the node as explored since the shortest path to this node has been found
        explored.append(curr_node)

        # Destination Reached - Store results to output
        if curr_node == Data.end_node:
            path = '->'.join(curr_path)
            print("Shortest path: " + path + '.')
            distance = Data.calculate_distance(curr_path)
            print("Shortest distance: " + str(distance) + '.')
            print("Total energy cost: " + str(curr_cost) + '.')
            print("Creating output file...")
            result_to_txt(file_name, path, distance, curr_cost)
            print("Number of path explored before finding shortest path: " + str(fail_count))
            return "A* COMPLETE"

        for neighbour in Data.graph[curr_node]:
            # Get the distance between the current node and its neighbour
            node_pair = curr_path[-1] + ',' + neighbour
            distance = Data.dist[node_pair]
            # G(n)
            new_distance = distance + curr_distance
            # Energy cost of neighbour
            cost = Data.cost[node_pair]
            # New total energy cost after adding cost of neighbour
            new_cost = cost + curr_cost
            # Add to dictionary containing f(n) values of nodes explored if not explored or a shorter distance was found
            if (neighbour not in explored) or (new_distance < distance_so_far[neighbour]) or (has_energy_constraint and cost_dict[neighbour] > new_cost):
                distance_so_far[neighbour] = new_distance
                # Create a new path using the current path it is exploring
                new_path = list(curr_path)
                # Add neighbour of the current node to newly created path
                new_path.append(neighbour)
                # h(n) of neighbour
                heuristic_value = get_heuristic(Data, neighbour, weight)
                # f(n) = g(n) + h(n)
                new_distance += heuristic_value
                # Check if there is energy constraint
                if has_energy_constraint:
                    # Check if cost is within the energy constraint
                    if new_cost <= Data.energy_budget:
                        paths.put((new_distance, new_cost, new_path))
                        cost_dict[neighbour] = new_cost
                    # If path fails energy constraint
                    else:
                        fail_count += 1
                # If there is no energy constraint, check is not needed
                else:
                    paths.put((new_distance, new_cost, new_path))
                    cost_dict[neighbour] = new_cost

    # No valid path if no more vertices are in the Priority Queue
    print("A* FAILED")
    return "No path found from " + Data.start_node + " to " + Data.end_node

# Heuristic Function - Straight line distance from node to end node
def get_heuristic(data, vertex, weight):
    x_curr, y_curr = data.coord[vertex]
    x_end, y_end = data.coord[data.end_node]
    # Differences in x-coordinates and y-coordinates
    x_diff = x_end - x_curr
    y_diff = y_end - y_curr
    return weight * (sqrt(x_diff ** 2 + y_diff ** 2))

