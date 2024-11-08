import brickpi3
import time
import math

class Motion:
    def __init__(self):
        """
        Initialize a Motion object with motors and factors
        """
        self.BP = brickpi3.BrickPi3()
        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.left_factor = self.right_factor = 1, 1
        self.reset_encoders()
        self.stop()
        
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.wheel_diameter = 5.6
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.wheel_distance = 0 # the distance between the centers of both wheels
        self.left_ticks = self.BP.get_motor_encode(self.left_motor)
        self.right_ticks = self.BP.get_motor_encode(self.right_motor)


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

    def move_forward(self, speed=50, duration=1):
        """
        Moves forward for a certain duration
        """
        self.move(speed, speed, duration)

    def move_backward(self, speed=50, duration=1):
        """
        Moves backward for a certain duration
        """
        self.move(-speed, -speed, duration)

    def turn_left(self, speed=50, duration=1):
        """
        Turns left for a certain duration
        """
        self.move(-speed, speed, duration)

    def turn_right(self, speed=50, duration=1):
        """
        Turns right for a certain duration
        """
        self.move(speed, -speed, duration)
        
    def go_forward(self, speed=50, distance=10):
        """
        Moves forward for a certain distance
        """
        target_ticks = distance / self.wheel_circumference * 360
        self._move_for_distance(speed, speed, target_ticks)
        
    def go_forward(self, speed=50, distance=10):
        """
        Moves backward for a certain distance
        """
        target_ticks = distance / self.wheel_circumference * 360
        self._move_for_distance(-speed, -speed, target_ticks)
        
    def turn_angle_right(self, speed=50, angle=90):
        """
        Turns right for a given angle
        """
        target_ticks = (angle / 360.0) * (math.pi * self.wheel_distance) / self.wheel_circumference * 360
        self._move_for_distance(-speed, speed, target_ticks)
        
    def turn_angle_left(self, speed=50, angle=90):
        """
        Turns left for a given angle
        """
        target_ticks = (angle / 360.0) * (math.pi * self.wheel_distance) / self.wheel_circumference * 360
        self._move_for_distance(speed, -speed, target_ticks)
    
    def stop(self):
        """
        Stops any movement
        """
        self.move(0, 0, 0)

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
        return (ticks / 360.0) * (self.wheel_circumference)
    
    def update_odometry(self):
        """
        Updates the position and angle of the robot
        """
        print("Not implemented")
        
    def get_position(self):
        """
        Returns the position and angle of the robot
        """
        return self.x, self.y, self.theta

# Private methods
    def _reset_encoders(self):
        """
        Resets encoders to 0
        """
        self.BP.reset_motor_encoder(self.left_motor)
        self.BP.reset_motor_encoder(self.right_motor)
        
    def _move(self, left_speed, right_speed, duration):
        """
        Controls both motors for a certain duration
        """
        self.BP.set_motor_power(self.left_motor, left_speed * self.left_factor)
        self.BP.set_motor_power(self.right_motor, right_speed * self.right_factor)
        time.sleep(duration)
        self.stop()
        
    def _move_for_distance(self, left_speed, right_speed, target_ticks):
        """
        Moves both motors until a given number of encoder ticks is reached
        """
        self._reset_encoders()
        
        while True:
            left_ticks = self.BP.get_motor_encoder(self.left_motor)
            right_ticks = self.BP.get_motor_encoder(self.right_motor)
            
            if abs(left_ticks) >= target_ticks and abs(right_ticks) >= target_ticks:
                break

            self.BP.set_motor_power(self.left_motor, left_speed * self.left_factor)
            self.BP.set_motor_power(self.right_motor, right_speed * self.right_factor)
        
        self.stop()