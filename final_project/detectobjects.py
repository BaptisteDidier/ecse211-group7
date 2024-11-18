from utils import sound
from utils.brick import TouchSensor, EV3UltrasonicSensor, EV3GyroSensor, wait_ready_sensors, reset_brick
from time import sleep, time
import math
import itertools
import Motion
from threading import Thread
import TSP


US_SENSOR_DATA_FILE = "../data_analysis/get_nodes_value.csv"
global node_list
node_list= []
global tuple_list
node_number = 1
TOUCH_SENSOR = TouchSensor(1)
US_SENSOR = EV3UltrasonicSensor(2)
GYRO_SENSOR = EV3GyroSensor(4)


def run_in_background(action):
    """
    Runs the given function in the background of the main thread
    """
    Thread(target=action, daemon=True).start()

def initial_movement():
    """
    Does the initial 90 degrees sweep to identify all the cubes to go to based on ultrasonic sensor's data
    """
    Motion.turn(50, 90, 'left')  
    Motion.turn(50, 90, 'right')  
    time.sleep(0.5)


def generate_node_edges(node_numbers):
    edges = list(itertools.combinations(node_numbers, 2))
    return edges

def collect_data_from_sweep(cur_value, node_list): # SET  node_number to 0 first
    output_file = open(US_SENSOR_DATA_FILE, "w")
    new_value = US_SENSOR.get_value()
    # we need to use the gyro sensor and not the ultrasonic sensor to get the angle of the sweep (from 0 to 90) 
    angle = GYRO_SENSOR.get_abs_measure()
    if new_value +0.5 < cur_value or new_value -0.5 > cur_value:
        print("invalid: same cube")
    elif new_value <= 30: #SET THIS BACK TO 120 AFTER
        output_file.write(f"{node_number}, {new_value}, {angle}\n")
        node_number += 1
        node_list.append(node_number)
        cur_value = new_value

def add_edge_weight_tuple(node1, node2, width, edge_weight_list, nodelist): #adds a new tuple element in list that will be used in graph and returns it
    if node1 not in nodelist:
        nodelist.append(node1)
    if node2 not in nodelist:
        nodelist.append(node2)
    new_tuple = (node1, node2, width)
    edge_weight_list.append(new_tuple)
    return new_tuple


def compute_weight(node1, node2, length1, length2, angle):
    if node1 == node2 and (length2 == None or length2 == 0):
        return length1
    elif node2 == node1 and (length1 == None or length1 == 0):
        return length2
    else:
        new_edge_width = math.sqrt((length1)**2+(length2)**2 - 2 * length1 * length2 * math.cos(angle))
        return new_edge_width
    
def convert_node_data_tuple(node_list, tuple_list):
    output_file = open(US_SENSOR_DATA_FILE, "r")
    edges = generate_node_edges(node_list)
    for edge in edges: #this is tuple value ex (0,1)
        for node1, node2 in edge:
            if node1 == output_file[0]:
                length1, angle1 = output_file[1], output_file[2]
                if node2 == output_file[0]:
                    length2, angle2 = output_file[1], output_file[2]
                    new_length =compute_weight(node1, node2,length1,length2, math.abs(angle1-angle2))              
                    add_edge_weight_tuple(node1, node2,new_length,tuple_list, node_list)  
    return tuple_list



if __name__ == "__main__":
    wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.
    print("Done waiting.")
    node_number = 0
    node_list = []
    tuple_list = tuple()
    first_value = US_SENSOR.get_value()
    run_in_background(initial_movement)
    while (True):
        collect_data_from_sweep(first_value, node_list)
        if TOUCH_SENSOR.is_pressed():
            break
    final = convert_node_data_tuple(node_list,tuple_list)
    graph = TSP.generate_complete_graph(node_list,final)
    tour_list_optimized = TSP.nearest_neighbor_tsp(graph,0) # now that we have the order of nodes, go to em




    




