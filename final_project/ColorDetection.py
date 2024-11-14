from Ressources import block_color_sensor


# Public methods
def get_normalized_rgb():
    """
    Returns the normalized values of the fixed color sensor
    """
    array = block_color_sensor.get_rgb()
    
    if any(value is None or value == 0 for value in array):
        print("Invalid reading")
        return [0, 0, 0]
    
    total = sum(array)
    return [round(c / total) for c in array]
    
def is_valid_block(array):
    """
    Compares an array of normalized rgb values to see whether it is valid
    """
    if any(value is None or value == 0 for value in array):
        print("Invalid reading")
        return False
    
    if (array[0] > 35 or array[0] < 55): # need to test to find the correct intervals
        return False
    
    if (array[1] > 35 or array[1] < 55): # need to test to find the correct intervals
        return False
    
    if (array[2] > 35 or array[2] < 55): # need to test to find the correct intervals
        return False
    
    return True