from Resources import *
import Motion
from Grabbing import open_gate, close_gate, is_valid_block, collect_block, eject
from time import sleep


def robot_constant_move():
    left_motor.set_power(30)
    right_motor.set_power(30)

def check_distance():
    value = ultrasonic_sensor.get_cm()
    print(value)
    if value > 8 and value < 20:
        Motion.stop()
        Motion.turn(30,90, "left")
        robot_constant_move()

def check_distance2():
    while not (8 < ultrasonic_sensor.get_cm() < 30):
        Motion.move(40, 2)
    Motion.turn(30, -90, "left")
    check_distance2()

def main():
    initialize_components()
    left_motor.set_limits(30,360)
    right_motor.set_limits(30,360)
    robot_constant_move()
    while True:
        check_distance()

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
    
def is_water():
    rgb = ground_color_sensor.get_rgb()
    while (rgb[0] == 0) and (rgb[1] == 0) and (rgb[2] == 0) or (rgb[0] == None) or (rgb[1] == None) or (rgb[2] == None):
        rgb = ground_color_sensor.get_rgb()
    print(rgb)
        
    if (20 <= rgb[0] <= 35) and (25 <= rgb[1] <= 45) and (40 <= rgb[2] <= 85):
        return True
        
    return False

def is_trashcan():
    rgb = ground_color_sensor.get_rgb()
    while (rgb[0] == 0) and (rgb[1] == 0) and (rgb[2] == 0) or (rgb[0] == None) or (rgb[1] == None) or (rgb[2] == None):
        rgb = ground_color_sensor.get_rgb()
    print(rgb)
        
    if (190 <= rgb[0] <= 210) and (150 <= rgb[1] <= 170) and (25 <= rgb[2] <= 45):
       print("trashcan detected")
       return True
        
    return False

        
def check_water():
    iteration = 0
    #print(ultrasonic_sensor.get_cm())
    while True:
        
        print(ultrasonic_sensor.get_cm())
        
        if is_water():
            print("is water !!")
            Motion.move(40, iteration, 'backward')
            break
        
        Motion.move(40, 10, 'forward')
        iteration += 0.8
        distance = ultrasonic_sensor.get_cm()
        
        if 0 < distance < 5:
            print("is wall !!")
            Motion.move(40, 95, 'backward')
            break
        
        print("is not water")
        array = block_color_sensor.get_rgb()
        while any(value is None or value == 0 for value in array):
            print("incorrect reading")
            array = block_color_sensor.get_rgb()
        
        if (array[0] + array[1] + array[2]) > 65:
            print("cube under")
            if is_valid_block():
                print("cube valid")
                collect_block()
            else:
                print("cube invalid")
                Motion.move(40, iteration, 'backward')
                print("backward done")
                break
            
        if is_trashcan():
            print("it should be ejecting now")
            Motion.stop()
            Motion.move(40, 10, 'forward')
            
            eject()
            
            print("finished ejecting")
            break
        
        #run_in_background(zig_zag_color_sensor)
        
        time.sleep(0.5)
        Motion.turn(20, 20,'right')
        check_color_sensor()
        
        time.sleep(0.5)
        Motion.turn(20, -40,'left')
        check_color_sensor()
        
        time.sleep(0.5)
        Motion.turn(20, 20,'right')


def zig_zag_color_sensor():
    print("checking the color in the background")
    array = block_color_sensor.get_rgb()
    print(array)
    while any(value is None or value == 0 for value in array):
        print("incorrect reading")
        array = block_color_sensor.get_rgb()
    
    if (array[0] + array[1] + array[2]) > 65:
        print("cube under")
        if is_valid_block():
            print("cube valid")
            collect_block()
        else:
            print("cube invalid")
            Motion.move(40, iteration, 'backward')
            print("backward done")

def check_color_sensor():
    print("checking the color in the background")
    array = block_color_sensor.get_rgb()
    while any(value is None or value == 0 for value in array):
        print("incorrect reading")
        array = block_color_sensor.get_rgb()
    
        if (array[0] + array[1] + array[2]) > 65:
            print("cube under")
            if is_valid_block():
                print("cube valid")
                collect_block()
            else:
                print("cube invalid")
                Motion.move(40, iteration, 'backward')
                print("backward done")
            break

def check_turn(direction='right'):
    #iteration = 0
    while ultrasonic_sensor.get_cm() > 3:
        Motion.turn(40, 90, direction)
        #iteration += 1.5
        if is_water():
            Motion.move(40, -90, 'left' if direction == 'right' else 'right')
            break

if __name__ == "__main__":
    print("1")
    check_water()