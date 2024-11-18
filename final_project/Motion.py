from Resources import left_motor, right_motor, gyro_sensor
import time
import math

# Motion
left_factor = 1
right_factor = 1
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

# PID
Kp = 2.0
Ki = 0.0
Kd = 1.0


# Public methods
def calibrate(speed=50, distance=10):
    """
    Calibrate the motors to adjust for any speed discrepancies
    """
    global left_factor, right_factor
    move(speed, distance, 'forward')
        
    left = left_motor.get_encoder()
    right = right_motor.get_encoder()
    factor = min(left, right) / max(left, right)
    left_factor, right_factor = (factor, 1) if left > right else (1, factor)

def move(speed=50, distance=10, direction='forward'):
    """
    Moves for a given speed, distance and direction
    """
    if direction not in ['forward', 'backward']: 
        raise ValueError("Direction must be 'forward' or 'backward'")
    target_ticks = (distance * 360) / wheel_circumference
    _move_for_distance(speed if direction == 'forward' else -speed, 
                       speed if direction == 'forward' else -speed, 
                       target_ticks, "move")

def turn(speed=50, angle=90, direction='right'):
    """
    Turns to a given angle, at a given speed and direction
    """
    if direction not in ['right', 'left']: 
        raise ValueError("Direction must be 'right' or 'left'")
    target_ticks = (angle * wheel_distance) / wheel_diameter
    _move_for_distance(-speed if direction == 'right' else speed, 
                       speed if direction == 'right' else -speed, 
                       target_ticks, "turn")

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

        x += delta_distance * math.cos(theta)
        y += delta_distance * math.sin(theta)
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
    delta_theta = target_theta - current_theta
    delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
    turn_angle = math.degrees(delta_theta)
    turn(speed, abs(turn_angle), "right" if turn_angle >= 0 else "left")
    move(speed, _get_euclidean_distance(current_x, current_y, target_x, target_y), "forward")


# Private methods
def _move_for_distance(left_speed, right_speed, target_ticks, type="move"):
    """
    Moves both motors until a given number of encoder ticks is reached
    """
    initial_angle = gyro_sensor.get_value()[0]
    initial_left = left_motor.get_encoder()
    initial_right = right_motor.get_encoder()
    last_error = 0
    integral = 0

    left_motor.set_power(left_speed * left_factor)
    right_motor.set_power(right_speed * right_factor)

    while True:
        current_left = left_motor.get_encoder() - initial_left
        current_right = right_motor.get_encoder() - initial_right
        current_angle = gyro_sensor.get_angle()[0]
        
        error = initial_angle - current_angle
        derivative = error - last_error
        integral += error
        correction = Kp * error + Ki * integral + Kd * derivative
        
        left_motor.set_power(left_speed + correction)
        right_motor.set_power(right_speed - correction)
        last_error = error

        if type == 'move' and (abs(current_left) >= target_ticks and abs(current_right) >= target_ticks):
            break
        elif type == 'turn' and abs(current_angle - initial_angle) >= target_ticks:
            break

        time.sleep(0.01)

    stop()

def _get_euclidean_distance(current_x, current_y, target_x, target_y):
    """
    Returns the distance between two points
    """
    return ((target_x - current_x) ** 2 + (target_y - current_y) ** 2) ** 0.5
