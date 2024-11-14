from Resources import gate_motor, block_color_sensor

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
    
def get_normalized_rgb():
    """
    Returns the normalized values of the fixed color sensor
    """
    array = list()
    for i in range(5):
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
    if any(value is None or value == 0 for value in array):
        print("Invalid reading")
        return False
    
    if (array[0] < 35 or array[0] > 55): # need to test to find the correct intervals
        return False
    
    if (array[1] < 35 or array[1] > 55): # need to test to find the correct intervals
        return False
    
    if (array[2] < 35 or array[2] > 55): # need to test to find the correct intervals
        return False
    
    return True