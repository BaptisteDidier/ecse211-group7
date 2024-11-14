from utils.brick import *

# Initialize every IO device here

# Motors
left_motor = Motor("A")
right_motor = Motor("B")
gate_motor = Motor("C")

# Sensors
ultrasonic_sensor = EV3UltrasonicSensor(1)
block_color_sensor = EV3ColorSensor(2)
scanning_color_sensor = EV3ColorSensor(3)