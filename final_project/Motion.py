import brickpi3
import time

class Motion:
    def __init__(self):
        """
        Initialize a Motion object with motors and factors
        Resets the motor encoders and stops motors
        """
        self.BP = brickpi3.BrickPi3()
        self.left_motor = self.BP.PORT_A
        self.right_motor = self.BP.PORT_B
        self.left_factor = self.right_factor = 1
        self.reset_encoders()
        self.stop()


# Public methods
    def initialize(self):
        """
        Calibrate the motors to adjust for any speed discrepancies
        """
        self.BP.set_motor_power(self.left_motor, 50)
        self.BP.set_motor_power(self.right_motor, 50)
        time.sleep(1)
        
        left = self.BP.get_motor_encoder(self.left_motor)
        right = self.BP.get_motor_encoder(self.right_motor)
        factor = min(left, right) / max(left, right)
        self.left_factor, self.right_factor = (factor, 1) if left > right else (1, factor)

    def move_forward(self, speed, duration):
        """
        Moves forward for a certain duration
        """
        self.move(speed, speed, duration)

    def move_backward(self, speed, duration):
        """
        Moves backward for a certain duration
        """
        self.move(-speed, -speed, duration)

    def turn_left(self, speed, duration):
        """
        Turns left for a certain duration
        """
        self.move(-speed, speed, duration)

    def turn_right(self, speed, duration):
        """
        Turns right for a certain duration
        """
        self.move(speed, -speed, duration)

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