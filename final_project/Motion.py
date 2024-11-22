from Resources import left_motor, right_motor, gyro_sensor, sweeping_motor, scanning_color_sensor
import time
import math
from Multithread import run_in_background

# Motion
angle_tolerance = 2
left_motor.set_limits(100, 1440)
right_motor.set_limits(100, 1440)
left_motor.reset_encoder()
right_motor.reset_encoder()

# Odometry
x = 0.0
y = 0.0
theta = 0.0
wheel_diameter = 4.2
wheel_circumference = math.pi * wheel_diameter
wheel_distance = 7.83

# PID (adjust if needed)
Kp = 0.04
Ki = 0.0
Kd = 0.01

# Sweeping
sweeping_motor.set_limits(50, 360)
sweeping_motor.reset_encoder()

class PIDController:
    def __init__(self, kp=Kp, ki=Ki, kd=Kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.last_error = 0
        self.integral = 0


# Public methods
#pidController = PIDController()

def move(speed=50, distance=10, direction='forward'):
    """
    Moves for a given speed, distance and direction
    """
    if direction not in ['forward', 'backward']: 
        raise ValueError("Direction must be 'forward' or 'backward'")
    target_ticks = (distance * 360) / wheel_circumference
    initial_left = left_motor.get_encoder()
    initial_right = right_motor.get_encoder()
    
    sweeping = run_in_background(sweep)
    
    while sweeping.is_alive():
        current_left = left_motor.get_encoder() - initial_left
        current_right = right_motor.get_encoder() - initial_right

        if current_left != 0 and current_right != 0:
            left_factor, right_factor = (1, current_left / current_right) if current_left <= current_right else (current_right / current_left, 1)
        else:
            left_factor, right_factor = 1, 1
       
        if direction == 'forward':
            left_motor.set_power(-speed*left_factor)
            right_motor.set_power(-speed*right_factor)
        else:
            left_motor.set_power(speed*left_factor)
            right_motor.set_power(speed*right_factor)
        
        if min(abs(current_left), abs(current_right)) >= target_ticks:
            break
        
        time.sleep(0.1)

    stop()

def turn(speed=50, angle=90, direction='right'):
    """
    Turns to a given angle, at a given speed and direction
    """
    if direction not in ['right', 'left']: 
        raise ValueError("Direction must be 'right' or 'left'")
    target_ticks = (angle * wheel_distance) / wheel_diameter
    initial_left = left_motor.get_encoder()
    initial_right = right_motor.get_encoder()
    sweeping = run_in_background(sweep)

    while sweeping.is_alive():
        current_left = left_motor.get_encoder() - initial_left
        current_right = right_motor.get_encoder() - initial_right
        
        if current_left != 0 and current_right != 0:
            left_factor, right_factor = (1, abs(current_left / current_right)) if abs(current_left) <= abs(current_right) else (abs(current_right / current_left), 1)
        else:
            left_factor, right_factor = 1, 1
            
        if direction == 'right':
            left_motor.set_power(-speed*left_factor)
            right_motor.set_power(speed*right_factor)
        else:
            left_motor.set_power(speed*left_factor)
            right_motor.set_power(-speed*right_factor)
        
        if min(abs(current_left), abs(current_right)) >= target_ticks:
            break
 
        time.sleep(0.1)

    stop()

def stop():
    """
    Stops any movement
    """
    left_motor.set_power(0)
    right_motor.set_power(0)

def encoder_to_distance(ticks):
    """
    Convert encoder ticks to distance in cm
    """
    return (ticks / 360.0) * wheel_circumference

def odometry(sampling_rate=0.1):
    """
    Updates the position and angle of the robot
    """
    global x, y, theta
    left_ticks = left_motor.get_encoder()
    right_ticks = right_motor.get_encoder()

    while True:
        new_left_ticks = left_motor.get_encoder()
        new_right_ticks = right_motor.get_encoder()

        delta_left = new_left_ticks - left_ticks
        delta_right = new_right_ticks - right_ticks

        delta_left_distance = encoder_to_distance(delta_left)
        delta_right_distance = encoder_to_distance(delta_right)

        delta_distance = (delta_left_distance + delta_right_distance) / 2.0
        delta_theta = (delta_right_distance - delta_left_distance) / wheel_distance

        x -= delta_distance * math.cos(math.radians(theta))
        y -= delta_distance * math.sin(math.radians(theta))
        theta += math.degrees(delta_theta) 
        theta = theta % 360

        left_ticks, right_ticks = new_left_ticks, new_right_ticks
        time.sleep(sampling_rate)

def get_position():
    """
    Returns the position and angle of the robot
    """
    return x, y, theta

def move_to(target_x, target_y, speed=50):
    """
    Moves to a certain position with given speed
    """
    current_x, current_y, current_theta = get_position()
    target_theta = math.atan2(target_y - current_y, target_x - current_x)
    delta_theta = (target_theta - current_theta + math.pi) % (2 * math.pi) - math.pi
    turn_angle = math.degrees(delta_theta)
    turn(speed, abs(turn_angle), "right" if turn_angle >= 0 else "left")
    move(speed, get_euclidean_distance(current_x, current_y, target_x, target_y), "forward")

def get_euclidean_distance(current_x, current_y, target_x, target_y):
    """
    Returns the distance between two points
    """
    return ((target_x - current_x) ** 2 + (target_y - current_y) ** 2) ** 0.5

def sweep():
    """
    Make the sweeping motion until blue is sampled
    """
    while True:
        sweeping_motor.set_power(20)
        while (sweeping_motor.get_encoder() < 90):
            rgb = get_normalized_value()
            if (40 < rgb[0] < 70) and (40 < rgb[1] < 80) and (125 < rgb[2] < 170):
                angle = sweeping_motor.get_encoder()
                sweeping_motor.set_power(0)
                sweeping_motor.set_position_relative(-angle)
                return angle
                
        sweeping_motor.set_power(-20)
        while(sweeping_motor.get_encoder() > -90):
            rgb = get_normalized_value()
            if (40 < rgb[0] < 70) and (40 < rgb[1] < 80) and (125 < rgb[2] < 170):
                angle = sweeping_motor.get_encoder()
                sweeping_motor.set_power(0)
                sweeping_motor.set_position_relative(-angle)
                return angle
        
def get_normalized_value():
    """
    Detects the normalized color of the ground
    """
    rgb = scanning_color_sensor.get_rgb()

    if any(value is None or value == 0 for value in rgb):
        return [0, 0, 0]
    
    total = sum(rgb)  
    return [round(255 * c / total, 0) for c in rgb]

def explore():
    while True:
        move(50, 100)
        turn(50, 90, 'right')
        move(50, 5)
        turn(50, 90, 'right')
        move(50, 100)
        turn(50, 90, 'left')
        move(50, 5)
        turn(50, 90, 'right')