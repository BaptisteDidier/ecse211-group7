import brickpi3
import time

class Motion:
    def __init__(self, left_motor_port=brickpi3.BrickPi3.PORT_A, right_motor_port=brickpi3.BrickPi3.PORT_B):
        self.BP = brickpi3.BrickPi3()
        self.left_motor_port = left_motor_port
        self.right_motor_port = right_motor_port

    def move_forward(self, speed, duration):
        self.BP.set_motor_power(self.left_motor_port, speed)
        self.BP.set_motor_power(self.right_motor_port, speed)
        time.sleep(duration)
        self.stop()

    def turn_left(self, speed, duration):
        self.BP.set_motor_power(self.left_motor_port, -speed)
        self.BP.set_motor_power(self.right_motor_port, speed)
        time.sleep(duration)
        self.stop()

    def turn_right(self, speed, duration):
        self.BP.set_motor_power(self.left_motor_port, speed)
        self.BP.set_motor_power(self.right_motor_port, -speed)
        time.sleep(duration)
        self.stop()

    def stop(self):
        self.BP.set_motor_power(self.left_motor_port, 0)
        self.BP.set_motor_power(self.right_motor_port, 0)

    def reset(self):
        self.BP.reset_all()


