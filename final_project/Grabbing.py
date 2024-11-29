from Resources import gate_motor

# Global variables
motor_power = 100
motor_dps = 720
gate_motor.set_limits(motor_power, motor_dps)
gate_motor.reset_encoder()
     
# Public methods   
def open_gate():
    """
    Open the gate
    """
    gate_motor.set_position_relative(65)
        
def close_gate():
    """
    Close the gate
    """
    gate_motor.set_position_relative(-65)