from Resources import left_motor, right_motor, gyro_sensor, block_color_sensor, ground_color_sensor, sweeping_motor
import time
import math
import threading

# Motion
min_turn_speed = 15
left_motor.set_limits(100, 1440)
right_motor.set_limits(100, 1440)
left_motor.reset_encoder()
right_motor.reset_encoder()
stop_move = threading.Event()

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
    
    #sweeping = run_in_background(sweep)
    
    while not stop_move.is_set():
        current_left = left_motor.get_encoder() - initial_left
        current_right = right_motor.get_encoder() - initial_right
        
        current_angle = gyro_sensor.get_abs_measure()
        correction = pidController.compute(initial_angle, current_angle)
        #print(current_left, current_right, current_angle, correction)

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
    #sweeping = run_in_background(sweep)

    while not stop_move.is_set():
        current_angle = gyro_sensor.get_abs_measure() - initial_angle
        print(current_angle)
        modular_speed = max(min_turn_speed, speed*((angle-current_angle)/angle)) # to slow down at the end of a turn
        
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

def sweep(angle1, angle2, delay=0.05):
    """
    Make the sweeping motion
    """
    print("1")
    while True:
        sweeping_motor.set_position_relative(angle1 - sweeping_motor.get_encoder())
        while abs(sweeping_motor.get_encoder() - angle1) > 2:  
            time.sleep(0.01)
        
        time.sleep(delay)

        sweeping_motor.set_position_relative(angle2 - sweeping_motor.get_encoder())
        while abs(sweeping_motor.get_encoder() - angle2) > 2:
            time.sleep(0.01)

        time.sleep(delay)
        
    sweeping_motor.set_power(0)
    sweeping_motor.set_position_relative(-sweeping_motor.get_encoder())
        
def get_normalized_value():
    """
    Detects the normalized color of the ground
    """
    rgb = block_color_sensor.get_rgb()

    if any(value is None or value == 0 for value in rgb):
        return [0, 0, 0]
    
    total = sum(rgb)  
    return [round(255 * c / total, 0) for c in rgb]

def explore():
    print("not implemented")

def thread_move(speed=40, distance=70, direction='forward'):
    stop_move.clear()
    move_thread = threading.Thread(target=move, args=(speed, distance, direction))
    move_thread.start()
    return move_thread

def thread_turn(speed=40, angle=90, direction='right'):
    stop_move.clear()
    turn_thread = threading.Thread(target=turn, args=(speed, angle, direction))
    turn_thread.start()
    return turn_thread

def thread_sweep(angle1=0, angle2=-90):
    stop_move.clear()
    sweep_thread = threading.Thread(target=sweep, args=(angle1, angle2))
    sweep_thread.start()
    return sweep_thread
#def sweep_for_cube(self):
       # to be done 

   # def reorient_and_pickup_cube(self):
       
       # detected_angle = self.sweep_for_cube()  
        #if detected_angle is not None:
          #  print(f"Cube detected at angle {detected_angle} degrees")
            
           # if detected_angle > 0:
             #   self.turn(40, detected_angle, "right")
            #else:
              #  self.turn(40, -detected_angle, "left")

            
           # print("Moving to pick up the cube...")
          #  self.collect_block()
       # else:
           # print("No cube detected in sweep.")
def detect_cubes():
    """
    Moves the robot and make the sweep until a valid block is seen and return its position relative to the robot
    """
    move_thread = thread_move()
    sweep_thread = thread_sweep()
    
    while True:
        intensity = block_color_sensor.get_rgb()
        if sum(intensity) > 65:
            angle = sweeping_motor.get_encoder()
            rgb = get_normalized_value()
            print(rgb)
            print("1")
            stop_move.set()
            #sweep_thread.join()
            print("2")
                
            if ((160 <= rgb[0] <= 205) and (30 <= rgb[1] <= 75) and (15 <= rgb[2] <= 35)) or \
               ((135 <= rgb[0] <= 175) and (70 <= rgb[1] <= 110) and (0 < rgb[2] <= 20)):# Orange or Yellow
                print("valid")
                sweeping_motor.set_position(-angle) 
                return angle
    
            else:
                print("invalid")
                sweeping_motor.set_position(-angle) 
                return None
            
        time.sleep(0.01)   
    
