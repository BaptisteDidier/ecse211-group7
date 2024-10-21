from utils import sound
from utils.brick import TouchSensor, wait_ready_sensors, reset_brick
import brickpi3
import time
from threading import Thread

# Constants
INIT_TIME = 1
POWER_LIMIT = 200
SPEED_LIMIT = 720
SOUND1 = sound.Sound(duration=0.7, pitch="C7", volume=100)
SOUND2 = sound.Sound(duration=0.7, pitch="D7", volume=100)
SOUND3 = sound.Sound(duration=0.7, pitch="E7", volume=100)
SOUND4 = sound.Sound(duration=0.7, pitch="F7", volume=100)
DATA_FILE = "../integration_test.csv"

print("Program start.\nWaiting for sensors to turn on...")

# Sensors
BP = brickpi3.BrickPi3()
MOTOR = BP.PORT_B
TOUCH_SENSORS = [TouchSensor(1), TouchSensor(2), TouchSensor(3), TouchSensor(4)]

wait_ready_sensors(True)
print("Done initializing")

# Methods
def play_sound(sound):
    """
    Plays a sound and waits for it to finish
    """
    sound.play()
    sound.wait_done()
    
def get_states():
    """
    Transforms the states of the touch sensors into a string
    """
    return ''.join('1' if sensor.is_pressed() else '0' for sensor in TOUCH_SENSORS)

def run_in_background(action):
    """
    Runs the given function in the background of the main thread
    """
    Thread(target=action, daemon=True).start()

def drumming_mechanism():
    """
    Initializes the motor and makes the drumming sound
    """
    BP.set_motor_limits(MOTOR, POWER_LIMIT, 400)
    BP.set_motor_position(MOTOR, 0)
    print("initialized motors")
    time.sleep(2)
    while True:
        BP.set_motor_position_relative(MOTOR, -90)
        time.sleep(0.3)
        BP.set_motor_position_relative(MOTOR, 90)
        time.sleep(0.3)

def main():
    """
    Handles the main loop for the system
    """
    output_file = open(DATA_FILE, "w")
    drumming_start = False
    
    try:
        while True:
            states = get_states()
            position = BP.get_motor_encoder(MOTOR)
            
            output_file.write(f"state: {states} ; position: {position}\n")

            if states == "1100" and not drumming_start:
                run_in_background(drumming_mechanism)
                drumming_start = True
                
            if states == "1111":
                break
            
            if states == "1000":
                play_sound(SOUND1)
                print("1")
                
            if states == "0100":
                play_sound(SOUND2)
                print("2")
                
            if states == "0010":
                play_sound(SOUND3)
                print("3")
                
            if states == "0001":
                play_sound(SOUND4)
                print("4")
                
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Done")
        output_file.close()
        reset_brick()

if __name__ == "__main__":
    BP.offset_motor_encoder(MOTOR, BP.get_motor_encoder(MOTOR))
    BP.set_motor_limits(MOTOR, POWER_LIMIT, SPEED_LIMIT)
    BP.set_motor_power(MOTOR, 0)
    main()
