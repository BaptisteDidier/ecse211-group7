from Resources import gate_motor, block_color_sensor
from Motion import move
from CostMap import *

# Global variables
motor_power = 100
motor_dps = 720
gate_motor.set_limits(motor_power, motor_dps)
gate_motor.reset_encoder()
     
     
# Public methods   
def open():
    """
    Open the gate
    """
    gate_motor.set_position_relative(90)
        
def close():
    """
    Close the gate
    """
    gate_motor.set_position_relative(-90)
    
def get_normalized_rgb(number=5):
    """
    Returns the normalized values of the fixed color sensor
    """
    array = list()
    for _ in range(number):
        rgb = block_color_sensor.get_rgb()
        if any(value is None or value == 0 for value in rgb):
            print("Invalid reading")
            return [0, 0, 0]
        array.append(rgb)
    
    total = [sum(x) for x in zip(*array)]
    return [round(255 * c / sum(total), 0) for c in total]
    
def is_valid_block():
    """
    Compares an array of normalized rgb values to see whether it is valid
    """
    array = get_normalized_rgb()
    while any(value is None or value == 0 for value in array):
        array = get_normalized_rgb()
    
    if (195 < array[0] < 205) and (30 < array[1] < 40) and (15 < array[2] < 20): # Yellow
        return True
    
    if (155 < array[0] < 175) and (70 < array[1] < 95) and (0 < array[2] < 10): # Orange
        return True
    
    return False

def collect_block():
    """
    Make the grabbing choice assuming that the robot is in the correct position
    """  
    if is_valid_block():
        collected_cubes += 1
        open()
        move(40, 11, 'forward')
        close()
        move(40, 11, 'backward')

    else:
        move(40, 5, 'backward')
        
def eject():
    """
    Places the cubes onto the garbage can
    """
    open()
    move(40, 33, 'backward')
    close()
    collected_cubes = 0