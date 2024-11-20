from Resources import left_motor, right_motor, gyro_sensor, sweeping_motor
import time
import math

# Motion
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
Kp = 0.5
Ki = 0.0
Kd = 0.0

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
pidController = PIDController()

def move(speed=50, distance=10, direction='forward'):
    """
    Moves for a given speed, distance and direction
    """
    if direction not in ['forward', 'backward']: 
        raise ValueError("Direction must be 'forward' or 'backward'")
    target_ticks = (distance * 360) / wheel_circumference
    initial_angle = gyro_sensor.get_abs_measure()
    initial_left = left_motor.get_encoder()
    initial_right = right_motor.get_encoder()
    pidController.reset()

    if direction == 'forward':
        left_motor.set_power(-speed)
        right_motor.set_power(-speed)
    else:
        left_motor.set_power(speed)
        right_motor.set_power(speed)
    
    while True:
        current_left = left_motor.get_encoder() - initial_left
        current_right = right_motor.get_encoder() - initial_right
        current_angle = gyro_sensor.get_abs_measure() 
        correction = pidController.compute(initial_angle, current_angle)
                
        if direction == 'forward':
            left_motor.set_power(-speed + correction)
            right_motor.set_power(-speed - correction)
        else:
            left_motor.set_power(speed - correction)
            right_motor.set_power(speed + correction)
        
        if min(abs(current_left), abs(current_right)) >= target_ticks:
            break
        
        time.sleep(0.01)

    stop()

def turn(speed=50, angle=90, direction='right'):
    """
    Turns to a given angle, at a given speed and direction
    """
    if direction not in ['right', 'left']: 
        raise ValueError("Direction must be 'right' or 'left'")
    pidController.reset()

    if direction == 'right':
        left_motor.set_power(-speed)
        right_motor.set_power(speed)
        target_angle = (gyro_sensor.get_abs_measure() + angle) % 360
    else:
        left_motor.set_power(speed)
        right_motor.set_power(-speed)
        target_angle = (gyro_sensor.get_abs_measure() - angle) % 360

    while True:
        current_angle = gyro_sensor.get_abs_measure()
        correction = pidController.compute(target_angle, current_angle)
        
        if direction == 'right':
            left_motor.set_power(-(speed - correction))
            right_motor.set_power(speed + correction)
        else:
            left_motor.set_power(speed + correction)
            right_motor.set_power(- (speed - correction))
        
        angle_difference = (current_angle - target_angle + 180) % 360 - 180
        if abs(angle_difference) <= 0.2:
            break
 
        time.sleep(0.01)

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

        x += delta_distance * math.cos(theta)
        y += delta_distance * math.sin(theta)
        delta_theta = gyro_sensor.get_abs_measure() - theta
        theta += delta_theta

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
    while True:
        sweeping_motor.set_position_relative(180)
        sweeping_motor.set_position_relative(-180)
        
        #sweeping_motor.set_power(20)
        #sleep(1.5)
        #sweeping_motor.set_power(-20)
        #sleep(1.5)
        