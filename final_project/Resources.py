from utils.brick import *
from utils.brick import wait_ready_sensors

# Initialize every IO device here

# Motors
left_motor = Motor("A")
right_motor = Motor("B")
gate_motor = Motor("C")

# Sensors
ultrasonic_sensor = EV3UltrasonicSensor(1)
block_color_sensor = EV3ColorSensor(2)
scanning_color_sensor = EV3ColorSensor(3)

def initialize_components():
    """ 
    Initializes all devices
    """
    wait_ready_sensors(True)
    print("Done initializing")