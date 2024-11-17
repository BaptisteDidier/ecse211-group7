import matplotlib.pyplot as plt
import random
import networkx as nx



def generate_complete_graph(num_nodes, weight_range=(1,100)): 
    # makes a graph using the number of nodes and an arbitrary weight. For our project we need to get the edge weight values in between nodes
    G = nx.complete_graph(num_nodes)
    for u, v in G.edges:
        G.edges[u,v]['weight'] = random.randint(*weight_range) # takes a random edge weight vakue for each edge in between each nodes
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
    G = generate_complete_graph(5)
    nearest_neighbor_tsp(G,0)
