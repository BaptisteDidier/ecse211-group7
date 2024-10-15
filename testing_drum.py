#Author: Tyler Vuong
import brickpi3
import time
from utils.brick import TouchSensor, EV3UltrasonicSensor, wait_read_sensors, reset_brick

BP = brickpi3.BrickPi3()
MOTOR = BP.PORT_A
sensor = EV3UltrasonicSensor(3)
POWER_LIMIT = 200
SPEED_LIMIT = 720
try:
    print("drumming mechanism test")
    try:
        BP = brickpi3.BrickPi3()
        BP.offset_motor_encoder(MOTOR, BP.get_motor_encoder(MOTOR))
        BP.set_motor_limits(MOTOR,POWER_LIMIT, SPEED_LIMIT)
        BP.set_motor_power(MOTOR,0)
        wait_sensors(True)
        print("Done waiting")
    except IOError as error:
        print(error)

    try: 
        BP.set_motor_limits(MOTOR, POWER_LIMIT,110)
        time.sleep(5)
        print(sensor.get_value())
        BP.set_motor_position_relative(MOTOR,90)
        time.sleep(5)
        BP.set_motor_position_relative(MOTOR,-90)
        time.sleep(4)
        print(sensor.get_value())
    
    except IOError as error:
        print(error)

except KeyboardInterrupt:
    BP.reset_all()








