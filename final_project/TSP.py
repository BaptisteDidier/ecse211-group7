import matplotlib.pyplot as plt
import random
import networkx as nx
import math

nodelist =[]

def compute_weight(node1, node2, length1, length2, angle):
    if node1 == node2 and length2 == None:
        return length1
    elif node2 == node1 and length1 == None:
        return length2
    else:
        new_edge_width = math.sqrt((length1)**2+(length2)**2 - 2 * length1 * length2 * math.cos(angle))
        return new_edge_width

def add_edge_weight_tuple(node1, node2, width, edge_weight_list): #adds a new tuple element in list that will be used in graph and returns it
    global nodelist
    if node1 not in nodelist:
        nodelist.append(node1)
    if node2 not in nodelist:
        nodelist.append(node2)
    new_tuple = (node1, node2, width)
    edge_weight_list.append(new_tuple)
    return new_tuple

# we need to somehow add into a tuple list (node1, node2, weight) and also nodes_list=[numbers in here]
def generate_complete_graph(nodes_list,edge_weight_list): 
    # makes a graph using the number of nodes and an arbitrary weight. For our project we need to get the edge weight values in between nodes
    G = nx.graph()
    G.add_nodes_from(nodes_list)
    G.add_weighted_edges_from(edge_weight_list)
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
    
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    #G = generate_complete_graph(5)
    nearest_neighbor_tsp(G,0)
