from Resources import *
import time
import math
import threading
from Grabbing import collect

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
sweeping_motor.set_limits(30, 280)
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

    while not stop_move.is_set():
        current_angle = gyro_sensor.get_abs_measure() - initial_angle
        modular_speed = max(min_turn_speed, speed*((angle-current_angle)/angle))
        
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

def sweep(angle1, angle2, delay=0.05):
    """
    Make the sweeping motion
    """
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
    sweeping_motor.set_power(0)
        
def get_normalized_value():
    """
    Detects the normalized color of the ground
    """
    rgb = block_color_sensor.get_rgb()

    if any(value is None or value == 0 for value in rgb):
        return [0, 0, 0]
    
    total = sum(rgb)  
    return [round(255 * c / total, 0) for c in rgb]

def thread_move(speed=20, distance=70, direction='forward'):
    """
    Create a moving forward thread
    """
    stop_move.clear()
    move_thread = threading.Thread(target=move, args=(speed, distance, direction))
    move_thread.start()
    return move_thread

def thread_turn(speed=40, angle=90, direction='right'):
    """
    Create a turning thread
    """
    stop_move.clear()
    turn_thread = threading.Thread(target=turn, args=(speed, angle, direction))
    turn_thread.start()
    return turn_thread

def thread_sweep(angle1=45, angle2=-45):
    """
    Create a sweeping thread
    """
    stop_move.clear()
    sweep_thread = threading.Thread(target=sweep, args=(angle1, angle2))
    sweep_thread.start()
    return sweep_thread

def detect_cubes():
    """
    Moves the robot and make the sweep until a valid block is seen and return its position relative to the robot
    """
    #move_thread = thread_move()
    sweep_thread = thread_sweep()
    
    while True:
        rgb = ground_color_sensor.get_rgb()
        if (20 <= rgb[0] <= 35) and (25 <= rgb[1] <= 45) and (40 <= rgb[2] <= 85): # Water detected
            return None

        distance = ultrasonic_sensor.get_cm()
        if 0 < distance < 5: # Wall detected
            return None
        
        intensity = block_color_sensor.get_rgb()
        if sum(intensity) > 65:
            angle = sweeping_motor.get_encoder()
            rgb = get_normalized_value()
            print(rgb)
            stop_move.set()
            reset_sweep()
            time.sleep(0.01)
            #sweep_thread.join()
                
            if ((160 <= rgb[0] <= 205) and (30 <= rgb[1] <= 75) and (15 <= rgb[2] <= 35)) or \
               ((120 <= rgb[0] <= 175) and (70 <= rgb[1] <= 120) and (0 < rgb[2] <= 30)):# Orange or Yellow
                print("valid")
                return angle
    
            else:
                print("invalid")
                return None
        
        time.sleep(0.01)

def orient_and_pickup():
    print("will start to pickup cubes")
    detected_angle = detect_cubes()
    stop_move.clear()
    
    if detected_angle is not None:
        print("angle was detected")
        print(detected_angle)
              
        if detected_angle > 0:
            turn(20, 10, "right")
            print("turn right")
        elif detected_angle < 0:
            turn(20, -10, "left")
            print("turn left")
            
        collect()
    else:
        print("no valid cube was detected")
    
            
    