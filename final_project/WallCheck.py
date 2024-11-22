from Resources import *
import Motion
from Multithread import run_in_background


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
    
    
def is_water():
    rgb = block_color_sensor.get_rgb()
    print(rgb)
        
    if (0 < rgb[0] < 10) and (0 < rgb[1] < 5) and (0 < rgb[2] < 5):
        return True
        
    return False
        
def check_water():
    iteration = 0
    while ultrasonic_sensor.get_cm() > 3:
        Motion.move(40, 1, 'forward')
        iteration += 1.5
        if is_water():
            Motion.move(40, iteration, 'backward')
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
    main()