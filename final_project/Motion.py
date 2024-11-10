import brickpi3
import time
import math

class Motion:
    def __init__(self):
        """
        Initialize a Motion object with motors and factors
        """
        # Motion
        self.BP = brickpi3.BrickPi3()
        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.left_factor = self.right_factor = 1, 1
        self.reset_encoders()
        self.stop()
        
        # Odometry
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.wheel_diameter = 5.6
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.wheel_distance = 0  # the distance between the centers of both wheels
        self.left_ticks = self.BP.get_motor_encoder(self.left_motor)
        self.right_ticks = self.BP.get_motor_encoder(self.right_motor)

        # Color detection
        # self.color_sensor = self.BP.PORT_1
        # self.BP.set_sensor_type(self.color_sensor, self.BP.SENSOR_TYPE.EV3_COLOR_COLOR)

# Public methods
    def calibrate(self, speed=50, duration=1):
        """
        Calibrate the motors to adjust for any speed discrepancies
        """
        self.BP.set_motor_power(self.left_motor, speed)
        self.BP.set_motor_power(self.right_motor, speed)
        time.sleep(duration)
        
        left = self.BP.get_motor_encoder(self.left_motor)
        right = self.BP.get_motor_encoder(self.right_motor)
        factor = min(left, right) / max(left, right)
        self.left_factor, self.right_factor = (factor, 1) if left > right else (1, factor)
        
    def move(self, speed=50, distance=10, direction='forward'):
        """
        Moves for a given speed, distance and direction
        """
        if direction not in ['forward', 'backward']: 
            raise ValueError("Direction must be 'forward' or 'backward'")
        target_ticks = distance / self.wheel_circumference * 360
        self._move_for_distance(speed if direction == 'forward' else -speed, 
                                speed if direction == 'forward' else -speed, 
                                target_ticks)
      
    def turn(self, speed=50, angle=90, direction='right'):
        """
        Turns to a given angle, at a given speed and direction
        """
        if direction not in ['right', 'left']: 
            raise ValueError("Direction must be 'right' or 'left'")
        target_ticks = (angle / 360.0) * (math.pi * self.wheel_distance) / self.wheel_circumference * 360
        self._move_for_distance(-speed if direction == 'right' else speed, 
                                 speed if direction == 'right' else -speed, 
                                 target_ticks)

    def stop(self):
        """
        Stops any movement
        """
        self._move_for_distance(0, 0, 0)

    def reset(self):
        """
        Resets the BrickPi
        """
        self.stop()
        self.BP.reset_all()

    def encoder_to_distance(self, ticks):
        """
        Convert encoder ticks to distance in cm
        """
        return (ticks / 360.0) * self.wheel_circumference
                
    def odometry(self, sampling_rate=0.1):
        """
        Updates the position and angle of the robot
        """
        left_ticks = self.BP.get_motor_encoder(self.left_motor)
        right_ticks = self.BP.get_motor_encoder(self.right_motor)
        
        while True:
            new_left_ticks = self.BP.get_motor_encoder(self.left_motor)
            new_right_ticks = self.BP.get_motor_encoder(self.right_motor)

            delta_left = new_left_ticks - left_ticks
            delta_right = new_right_ticks - right_ticks

            delta_left_distance = self.encoder_to_distance(delta_left)
            delta_right_distance = self.encoder_to_distance(delta_right)

            delta_distance = (delta_left_distance + delta_right_distance) / 2.0
            delta_theta = (delta_right_distance - delta_left_distance) / self.wheel_distance

            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)
            self.theta += delta_theta

            left_ticks, right_ticks = new_left_ticks, new_right_ticks
            time.sleep(sampling_rate)
        
    def get_position(self):
        """
        Returns the position and angle of the robot
        """
        return self.x, self.y, self.theta  
    
    def move_to(self, x, y, speed=50):
        """
        Moves to a certain position
        """
        current_x, current_y, current_theta = self.get_position()
        delta_theta = current_theta - self.theta
        
        self.turn(speed, delta_theta, "right" if delta_theta >= 180 else "left")
        self.move(speed, self._get_euclidean_distance(current_x, current_y, x, y), "forward")

# Private methods
    def reset_encoders(self):
        """
        Resets encoders to 0
        """
        self.BP.reset_motor_encoder(self.left_motor)
        self.BP.reset_motor_encoder(self.right_motor)
        
    def _move_for_distance(self, left_speed, right_speed, target_ticks):
        """
        Moves both motors until a given number of encoder ticks is reached
        """
        self.reset_encoders()
        
        while True:
            left_ticks = self.BP.get_motor_encoder(self.left_motor)
            right_ticks = self.BP.get_motor_encoder(self.right_motor)
            
            if abs(left_ticks) >= target_ticks and abs(right_ticks) >= target_ticks:
                break

            self.BP.set_motor_power(self.left_motor, left_speed * self.left_factor)
            self.BP.set_motor_power(self.right_motor, right_speed * self.right_factor)
        
        self.stop()
        
    def _get_euclidean_distance(self, current_x, current_y, target_x, target_y):
        """
        Returns the distance between two points
        """
        return ((target_x - current_x)**2 + (target_y - current_y)**2)**0.5
