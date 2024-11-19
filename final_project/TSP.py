import matplotlib.pyplot as plt
import random
import networkx as nx

# we need to somehow add into a tuple list (node1, node2, weight) and also nodes_list=[numbers in here]
def generate_complete_graph(nodes_list,edge_weight_list): 
    # makes a graph using the number of nodes and an arbitrary weight. For our project we need to get the edge weight values in between nodes
    G = nx.graph()
    G.add_nodes_from(nodes_list)
##Updated - Marleine
    # How about this? 
    for node1, node2, weight in edge_weight_list:
        if node1 in water_nodes or node2 in water_nodes:
            weight = float('inf')  # Mark water nodes as inaccessible
        if node1 in obstacle_nodes or node2 in obstacle_nodes:
            weight = float('inf')  # Mark obstacle nodes as inaccessible
        G.add_weighted_edges_from([(node1, node2, weight)])
    
   ## G.add_weighted_edges_from(edge_weight_list)
    return G

def plot_graph_step(G, tour, currentnode, pos):
    plt.clf()
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500)
    path_edges = list(zip(tour, tour[1:]))
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)
    nx.draw_networkx_edges(G, pos, nodelist=[currentnode], edge_color="green", node_size=500)

    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    plt.pause(0.5)

def calculate_tour_cost(G, tour):
    return sum(G[tour[i]][tour[i+1]]['weight'] for i in range(len(tour)-1))

def nearest_neighbor_tsp(G, startNode=None):
    if startNode is None:
        startNode = random.choice(list(G.nodes))
    
    pos = nx.spring_layout(G)
    plt.ion()
    plt.show()

    unvisited = set(G.nodes)
    unvisited.remove(startNode)

    tour =[startNode]
    current_node = startNode
    plot_graph_step(G, tour, current_node, pos)
    while unvisited:
        next_node = min(unvisited, key=lambda node: G[current_node][node]['weight'])
        unvisited.remove(next_node)
        tour.append(next_node)
        current_node=next_node
        plot_graph_step(G, tour, current_node, pos)
    
    tour.append(startNode)
    plot_graph_step(G, tour, current_node, pos)
    print(tour)
    tour_cost = calculate_tour_cost(G, tour)
    print(f"Construction heuristic tour cost: {tour_cost}")
    return tour
    
    
    plt.ioff()
    plt.show()
    
#updated - Marleine 
if __name__ == "__main__":
    #G = generate_complete_graph(5)
    #nearest_neighbor_tsp(G,0)
    ##print("1")
    # Initialize graphs and sensors
    node_list = [0, 1, 2, 3, 4]  
    edge_weight_list = [(0, 1, 10), (1, 2, 15), (2, 3, 10), (3, 4, 20)]  # to be changed 
    tsp = TSP(node_list, edge_weight_list)

    color_sensor = ColorSensor(INPUT_1)
    ultrasonic_sensor = UltrasonicSensor(INPUT_2)
    object_det = ObjectDet(color_sensor, ultrasonic_sensor)

    current_node = 0
    unvisited_nodes = set(node_list)

    print("Starting navigation")

    while unvisited_nodes:
        # Detect water and cube
        if object_det.detect_water(current_node):
            tsp.update_graph_for_water_or_obstacle(current_node)
        if object_det.detect_obstacle(current_node):
            tsp.update_graph_for_water_or_obstacle(current_node)
        # Recalculate the path to the next one/cube
        unvisited_nodes -= {current_node}
        if not unvisited_nodes:
            print("All nodes visited or inaccessible.")
            break
        path, next_node = tsp.recalculate_path(current_node, unvisited_nodes)
        if path is None:
            break


        print(f"Following path: {path}")
        for node in path[1:]:  # Skip the current node
            print(f"Moving to node {node}")
            # Insert motion control logic here (e.g., turn, move forward)
            current_node = node

        #  If all cubes collected
        if TOUCH_SENSOR.is_pressed():  # Add an ending condition based on the task
            print("Task completed!")
            break

    print("Navigation complete.")

