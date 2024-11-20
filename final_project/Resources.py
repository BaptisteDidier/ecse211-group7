from utils.brick import *

# Initialize every IO device here

# Motors
left_motor = Motor("A")
right_motor = Motor("B")
gate_motor = Motor("C")
sweeping_motor = Motor("D")

# Sensors
ultrasonic_sensor = EV3UltrasonicSensor(1)
block_color_sensor = EV3ColorSensor(2)
scanning_color_sensor = EV3ColorSensor(4)
gyro_sensor = EV3GyroSensor(3).set_mode("abs")
gyro_sensor.reset_measure()

def initialize_components():
    """ 
    Initializes all devices
    """
    wait_ready_sensors(True)
    print("Done initializing")
