from Resources import *
import time
import math
import threading
<<<<<<< Updated upstream
from Grabbing import collect


=======
import Grabbing
>>>>>>> Stashed changes
# Motion
wheel_diameter = 4.2
wheel_circumference = math.pi * wheel_diameter
wheel_distance = 7.83
min_turn_speed = 15
left_motor.set_limits(100, 1440)
right_motor.set_limits(100, 1440)
left_motor.reset_encoder()
right_motor.reset_encoder()
stop_move = threading.Event()

# PID
Kp = 0.5
Ki = 0.0
Kd = 0.0
dT = 0.05

# Sweeping
<<<<<<< Updated upstream
sweeping_motor.set_limits(30, 280)
=======
sweeping_motor.set_limits(40, 360)
>>>>>>> Stashed changes
sweeping_motor.reset_encoder()

class PIDController:
    """
    Create a PID Controller for motion
    """
    def __init__(self, kp=Kp, ki=Ki, kd=Kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error * dT
        derivative = (error - self.last_error) / dT
        self.last_error = error
        return self.kp * error + self.integral * self.ki + self.kd * derivative

    def reset(self):
        self.last_error = 0
        self.integral = 0

# Public methods
pidController = PIDController()

def move(speed=40, distance=50, direction='forward'):
    """
    Moves for a given speed, distance and direction
    """
    if direction not in ['forward', 'backward']: 
        raise ValueError("Direction must be 'forward' or 'backward'")
    target_ticks = (distance * 360) / wheel_circumference
    initial_left = left_motor.get_encoder()
    initial_right = right_motor.get_encoder()
    initial_angle = gyro_sensor.get_abs_measure()
    pidController.reset()
    
    while not stop_move.is_set():
        current_left = left_motor.get_encoder() - initial_left
        current_right = right_motor.get_encoder() - initial_right
        
        current_angle = gyro_sensor.get_abs_measure()
        correction = pidController.compute(initial_angle, current_angle)

        if direction == 'forward':
            left_motor.set_power(-speed - correction)
            right_motor.set_power(-speed + correction)
        else:
            left_motor.set_power(speed - correction)
            right_motor.set_power(speed + correction)
        
        if abs(current_left) >= target_ticks or abs(current_right) >= target_ticks:
            stop()
            break

        time.sleep(dT)

    stop()

def turn(speed=40, angle=90, direction='right'):
    """
    Turns to a given angle, at a given speed and direction
    """
    
    if direction not in ['right', 'left']: 
        raise ValueError("Direction must be 'right' or 'left'")
    initial_angle = gyro_sensor.get_abs_measure()
<<<<<<< Updated upstream
=======
    
    #sweeping = run_in_background(sweep)
>>>>>>> Stashed changes

    while not stop_move.is_set():
        
        current_angle = gyro_sensor.get_abs_measure() - initial_angle
<<<<<<< Updated upstream
        modular_speed = max(min_turn_speed, speed*((angle-current_angle)/angle))
=======
        #print(current_angle)
        modular_speed = max(min_turn_speed, speed*((angle-current_angle)/angle)) # to slow down at the end of a turn
>>>>>>> Stashed changes
        
        if direction == 'right':
            left_motor.set_power(-modular_speed)
            right_motor.set_power(modular_speed)
            if angle <= current_angle:
                stop()
                break
        else:
            left_motor.set_power(modular_speed)
            right_motor.set_power(-modular_speed)
            if angle >= current_angle:
                stop()
                break
 
        time.sleep(dT)
        
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

<<<<<<< Updated upstream
=======
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

>>>>>>> Stashed changes
def sweep(angle1, angle2, delay=0.05):
    """
    Make the sweeping motion
    """
    print("starts sweeping")
    while not stop_move.is_set():
        sweeping_motor.set_position(angle1 - sweeping_motor.get_encoder())
        while abs(sweeping_motor.get_encoder() - angle1) > 2:
            if stop_move.is_set():
                break
            time.sleep(0.01)
            
        if stop_move.is_set():
            print("stop move is set my lord")
            break
        
        time.sleep(delay)
        

        sweeping_motor.set_position(angle2 - sweeping_motor.get_encoder())
        while abs(sweeping_motor.get_encoder() - angle2) > 2:
            if stop_move.is_set():
                break
            time.sleep(0.01)
        
        if stop_move.is_set():
            break
        
        time.sleep(delay)

    sweeping_motor.set_position(0)
    print("position has been set to 0 in sweep")
    """this time sleep is absolutely necessary"""
    
    time.sleep(1) 
<<<<<<< Updated upstream
    sweeping_motor.set_power(0)
    time.sleep(1)

def reset_sweep():
    """
    Resets the sweeping motor to its original position
    """ 
    stop_move.set()
    sweeping_motor.set_position(-sweeping_motor.get_encoder())
    while abs(sweeping_motor.get_encoder()) > 2:
        time.sleep(0.01)
=======
>>>>>>> Stashed changes
    sweeping_motor.set_power(0)
    time.sleep(1)
    
        
def get_normalized_value():
    """
    Detects the normalized color of the ground
    """
    rgb = block_color_sensor.get_rgb()

    if any(value is None or value == 0 for value in rgb):
        return [0, 0, 0]
    
    total = sum(rgb)  
    return [round(255 * c / total, 0) for c in rgb]

<<<<<<< Updated upstream
def thread_move(speed=20, distance=70, direction='forward'):
    """
    Create a moving forward thread
    """
=======
def explore():
    print("not implemented")

def thread_move(speed=40, distance=95, direction='forward'):
>>>>>>> Stashed changes
    stop_move.clear()
    move_thread = threading.Thread(target=move, args=(speed, distance, direction), daemon=True)
    move_thread.start()
    return move_thread

def thread_turn(speed=40, angle=90, direction='right'):
    """
    Create a turning thread
    """
    stop_move.clear()
    turn_thread = threading.Thread(target=turn, args=(speed, angle, direction), daemon=True)
    turn_thread.start()
    return turn_thread

def thread_sweep(angle1=45, angle2=-45):
<<<<<<< Updated upstream
    """
    Create a sweeping thread
    """
=======
>>>>>>> Stashed changes
    stop_move.clear()
    sweep_thread = threading.Thread(target=sweep, args=(angle1, angle2), daemon=True)
    sweep_thread.start()
    return sweep_thread

def detect_cubes():
    """
    Moves the robot and make the sweep until a valid block is seen 
    and return its position relative to the robot or when it hits
    an obstacle
    """
    wall_distance = ultrasonic_sensor.get_cm() - 10
    move_thread = thread_move(28, wall_distance)
    sweep_thread = thread_sweep()
<<<<<<< Updated upstream
    
    while True:
        rgb = ground_color_sensor.get_rgb()
        if (20 <= rgb[0] <= 35) and (25 <= rgb[1] <= 45) and (40 <= rgb[2] <= 85): # Water detected
            stop_move.set()
            move_thread.join()
            sweep_thread.join()
            reset_sweep()
            return None

        distance = ultrasonic_sensor.get_cm()
        if 0 < distance < 5: # Wall detected
            stop_move.set()
            move_thread.join()
            sweep_thread.join()
            reset_sweep()
            return None
        
        intensity = block_color_sensor.get_rgb()
        if sum(intensity) > 65:
            angle = sweeping_motor.get_encoder()
            rgb = get_normalized_value()
            print(rgb)
            stop_move.set()
            move_thread.join()
            sweep_thread.join()
            reset_sweep()
            time.sleep(0.01)
                
            if ((160 <= rgb[0] <= 205) and (30 <= rgb[1] <= 75) and (15 <= rgb[2] <= 35)) or \
               ((120 <= rgb[0] <= 175) and (70 <= rgb[1] <= 120) and (0 < rgb[2] <= 30)):# Orange or Yellow
                print("valid")
                return angle
    
            else:
                print("invalid")
                return None
        
        time.sleep(0.01)
=======

    while True:       
        rgb = ground_color_sensor.get_rgb()
        if (20 <= rgb[0] <= 35) and (25 <= rgb[1] <= 45) and (40 <= rgb[2] <= 85):
            stop_move.set()
            move_thread.join()
            sweep_thread.join()
            reset_sweeping()
            print("RGB")
            print(rgb)
            return None
        
        if not move_thread.is_alive():
            stop_move.set()
            move_thread.join()
            sweep_thread.join()
            reset_sweeping()
            print("Distance")
            return None
        
        intensity = block_color_sensor.get_rgb()
        if sum(intensity) > 75:
            angle = sweeping_motor.get_encoder()
            rgb = get_normalized_value()
            print(rgb)
            stop_move.set()
            move_thread.join()
            sweep_thread.join()
            reset_sweeping()
            time.sleep(0.01)
  
            if ((160 <= rgb[0] <= 205) and (30 <= rgb[1] <= 75) and (15 <= rgb[2] <= 40)) or \
               ((120 <= rgb[0] <= 175) and (70 <= rgb[1] <= 120) and (0 < rgb[2] <= 30)):# Orange or Yello
                print("valid")
                return angle
    
            else:
                print("invalid")
                return None
 
        time.sleep(0.01)

def reset_sweeping():
    sweeping_motor.set_position(0)
    sweeping_motor.set_power(0)
>>>>>>> Stashed changes

def orient_and_pickup():
    print("will start to pickup cubes")
    detected_angle = detect_cubes()
    stop_move.clear()
    
    if detected_angle is not None:
        print("angle was detected")
        print(detected_angle)
<<<<<<< Updated upstream
              
        if detected_angle > 0:
            turn(20, 10, "right")
            print("turn right")
        elif detected_angle < 0:
            turn(20, -10, "left")
            print("turn left")
            
        collect()
    else:
=======
        
        
        if detected_angle > 0:
            turn(20, 10, "right")
            time.sleep(0.5)
            print("turn right")
        elif detected_angle < 0:
            turn(20, -10, "left")
            time.sleep(0.5)
            print("turn left")
            
        print("collect")
        move(30, 3, 'backward')
        Grabbing.open_gate()
        move(30, 10, 'forward')
        Grabbing.close_gate()
        move(30, 7, 'backward')
        print("collecting")
        
        if detected_angle > 0: # we may have to change this
            turn(20, -12, "left")
            time.sleep(0.5)
            
        elif detected_angle < 0:
            turn(20, 12, "right")
            time.sleep(0.5)
        
        orient_and_pickup()
        
    else:
        move(40, 98 - ultrasonic_sensor.get_cm(), 'backward')
>>>>>>> Stashed changes
        print("no valid cube was detected")
    
            
    