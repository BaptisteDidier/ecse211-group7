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


def main():
    initialize_components()
    left_motor.set_limits(30,360)
    right_motor.set_limits(30,360)
    robot_constant_move()
    while True:
        check_distance()
    
    



if __name__ == "__main__":
    main()