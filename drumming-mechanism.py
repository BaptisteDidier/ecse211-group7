#Author: Tyler Vuong
import brickpi3
import time
from utils.brick import TouchSensor, EV3UltrasonicSensor, wait_read_sensors, reset_brick

BP = brickpi3.BrickPi3() #initialize brickpi
MOTOR = BP.PORT_A
sensor = EV3UltrasonicSensor(3) 
TOUCH_SENSORS = [TouchSensor(1), TouchSensor(2), TouchSensor(3), TouchSensor(4)]

POWER_LIMIT = 200
SPEED_LIMIT = 720

def get_states():
    return ''.join('1' if sensor.is_pressed() else '0' for sensor in TOUCH_SENSORS)

def drum_loop():
    while True:
        BP.set_motor_limits(MOTOR, POWER_LIMIT,140) #CHANGE VALUES DEPENDENT
        time.sleep(2)
        BP.set_motor_position_relative(MOTOR,-90) #rotate motor down
        time.sleep(2) #wait in between
        BP.set_motor_position_relative(MOTOR,90) #rotate motor up
        time.sleep(2)
        if states == "1111":
            return False
    



try:
    print("drumming mechanism test")
    try:
        BP = brickpi3.BrickPi3() #set everything with limits and initialize sensors
        BP.offset_motor_encoder(MOTOR, BP.get_motor_encoder(MOTOR))
        BP.set_motor_limits(MOTOR,POWER_LIMIT, SPEED_LIMIT)
        BP.set_motor_power(MOTOR,0)
        wait_sensors(True)
        print("Done waiting")
    except IOError as error:
        print(error)

    try: 
        states = get_states()
        if states == '1000' or states == '0100' or states == '0010' or states == '0001':
            drum_loop()
    
    
    except IOError as error:
        print(error)

except KeyboardInterrupt:
    BP.reset_all()