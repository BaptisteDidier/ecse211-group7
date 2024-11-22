from Resources import ultrasonic_sensor
from Multithread import run_in_background
import Motion
import math

# Global variable
obstacles = list()
costmap = [[0 for _ in range(122)] for _ in range(122)]
tolerance = 3
collected_cubes = 0

# Methods
def get_obstacle_position():
    """
    Get the position of the obstacle in front depending on the current position
    """
    global obstacles
    distance = ultrasonic_sensor.get_cm()
    if distance is None:
        return None;
    
    x, y, theta = Motion.get_position()
    new_x = x + distance*math.cos(math.radians(theta))
    new_y = y + distance*math.sin(math.radians(theta))
    
    if not (0 < new_x < 122) or not (0 < new_y < 122): # if the sensor sees the walls
        return None
    
    if any(abs(node[0] - new_x) <= tolerance and abs(node[1] - new_y) <= tolerance for node in obstacles): # if the node already exists
        return None

    obstacles.append((new_x, new_y))
    return new_x, new_y

def add_obstacle(obstacle_position):
    """
    Add an obstacle to the costmap
    """
    if obstacle_position is None or any( not(0 < value < 122) for value in obstacle_position):
        print("Invalid obstacle")
        return;
 
    costmap[int(obstacle_position[0])][int(obstacle_position[1])] = 1
    
def scan(speed=50, angle=180):
    """
    Turns to assess where the obstacles are
    """
    action = run_in_background(Motion.turn(speed, angle, "right"))
    
    while action.is_alive():
        add_obstacle(get_obstacle_position())
        
def euclidean(current, target):
    """
    Returns the distance between two points
    """
    return ((target[0] - current[0]) ** 2 + (target[1] - current[1]) ** 2) ** 0.5

def mst_cost(nodes):
    """
    Calculate the MST cost for a set of nodes using Prim's algorithm
    """
    if len(nodes) < 2:
        return 0
    
    unvisited = nodes.copy()
    visited = [unvisited.pop()]
    mst_cost = 0
    
    # we search for the closest nodes from those in visited
    while unvisited:
        min_edge = float('inf')
        nearest_node = None
        for u in unvisited:
            for v in visited:
                distance = euclidean(u, v)
                if distance < min_edge:
                    min_edge = distance
                    nearest_node = v
                    
        if nearest_node is None:
            break
        
        visited.append(nearest_node)
        unvisited.remove(nearest_node)
        mst_cost += min_edge
        
    return mst_cost

def heuristic(current_position, unexplored_nodes, goal):
    """
    Heuristic for A* using MST and nearest distances to visit all the nodes 
    before reaching the goal
    """
    # if we visited every cube
    if not unexplored_nodes:
        return euclidean(current_position, goal)
    
    mst_cost = mst_cost(unexplored_nodes)
    nearest_to_current = min(euclidean(current_position, node) for node in unexplored_nodes)
    nearest_to_goal = min(euclidean(goal, node) for node in unexplored_nodes)
    return mst_cost + nearest_to_current + nearest_to_goal
