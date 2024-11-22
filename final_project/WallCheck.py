from Resources import *
import Motion



def robot_constant_move():
    left_motor.set_power(30)
    right_motor.set_power(30)

def check_distance():
    value = ultrasonic_sensor.get_cm()
    print(value)


def main():
    initialize_components()
    left_motor.set_limits(30,360)
    right_motor.set_limits(30,360)
    check_distance()
    



if __name__ == "__main__":
    main()