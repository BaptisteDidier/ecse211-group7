from Ressources import gate_motor

class Grabbing:
    def __init__(self):
        self.motor = gate_motor
        self.motor_power = 100
        self.motor_dps = 720
        self.motor.set_limits(self.motor_power, self.motor_dps)
        self.motor.reset_encoder()
        
    def open(self):
        """
        Open the gate
        """
        self.motor.set_position_relative(90)
        
    def close(self):
        """
        Close the gate
        """
        self.motor.set_position_relative(-90)