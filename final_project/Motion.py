from Ressources import left_motor, right_motor
import time
import math

class Motion:
    def __init__(self):
        """
        Initialize a Motion object with motors and factors
        """
        # Motion
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_factor = 1
        self.right_factor = 1
        self.left_motor.set_limits(100, 1440)
        self.right_motor.set_limits(100, 1440)
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_diameter = 4.2
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.wheel_distance = 7.83
        self.left_ticks = self.left_motor.get_encoder()
        self.right_ticks = self.right_motor.get_encoder()

        # Color detection
        # self.color_sensor =


# Public methods
    def calibrate(self, speed=50, distance=10):
        """
        Calibrate the motors to adjust for any speed discrepancies
        """
        self.move(speed, distance, 'forward')
        
        left = self.left_motor.get_encoder()
        right = self.right_motor.get_encoder()
        factor = min(left, right) / max(left, right)
        self.left_factor, self.right_factor = (factor, 1) if left > right else (1, factor)
        
    def move(self, speed=50, distance=10, direction='forward'):
        """
        Moves for a given speed, distance and direction
        """
        if direction not in ['forward', 'backward']: 
            raise ValueError("Direction must be 'forward' or 'backward'")
        target_ticks = (distance * 360) / self.wheel_circumference
        self._move_for_distance(speed if direction == 'forward' else -speed, 
                                speed if direction == 'forward' else -speed, 
                                target_ticks)
      
    def turn(self, speed=50, angle=90, direction='right'):
        """
        Turns to a given angle, at a given speed and direction
        """
        if direction not in ['right', 'left']: 
            raise ValueError("Direction must be 'right' or 'left'")
        target_ticks = (angle * self.wheel_distance) / self.wheel_diameter
        self._move_for_distance(-speed if direction == 'right' else speed, 
                                 speed if direction == 'right' else -speed, 
                                 target_ticks)
        self.left_motor.set_position()
        self.right_motor.set_position()

    def stop(self):
        """
        Stops any movement
        """
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)

    def encoder_to_distance(self, ticks):
        """
        Convert encoder ticks to distance in cm
        """
        return (ticks / 360.0) * self.wheel_circumference
                
    def odometry(self, sampling_rate=0.1):
        """
        Updates the position and angle of the robot
        """
        left_ticks = self.left_motor.get_encoder()
        right_ticks = self.right_motor.get_encoder()
        
        while True:
            new_left_ticks = self.left_motor.get_encoder()
            new_right_ticks = self.right_motor.get_encoder()

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
        Moves to a certain position with given speed
        """
        current_x, current_y, current_theta = self.get_position()
        target_theta = math.atan2(y - current_y, x - current_x)
        delta_theta = target_theta - math.radians(current_theta)
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        turn = math.degrees(delta_theta)
        self.turn(speed, abs(turn), "right" if turn >= 0 else "left")
        self.move(speed, self._get_euclidean_distance(current_x, current_y, x, y), "forward")


# Private methods
        
    def _move_for_distance(self, left_speed, right_speed, target_ticks):
        """
        Moves both motors until a given number of encoder ticks is reached
        """
        current_left = self.left_motor.get_encoder()
        current_right = self.right_motor.get_encoder()
        
        self.left_motor.set_power(left_speed * self.left_factor)
        self.right_motor.set_power(right_speed * self.right_factor)
            
        while True:
            left_ticks = self.left_motor.get_encoder() - current_left
            right_ticks = self.right_motor.get_encoder() - current_right
            
            if abs(left_ticks) >= target_ticks and abs(right_ticks) >= target_ticks:
                break

        self.stop()
        
    def _get_euclidean_distance(self, current_x, current_y, target_x, target_y):
        """
        Returns the distance between two points
        """
        return ((target_x - current_x)**2 + (target_y - current_y)**2)**0.5
