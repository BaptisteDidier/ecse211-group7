from Resources import gate_motor
from Motion import move

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
    gate_motor.set_position_relative(60)
        
def close_gate():
    """
    Close the gate
    """
    gate_motor.set_position_relative(-60)
    
def collect():
    move(40, 3, 'backward')
    open_gate()
    move(40, 10, 'forward')
    close_gate()
    move(40, 7, 'backward')
        
def eject():
    """
    Places the cubes onto the garbage can
    """
    open_gate()
    move(40, 33, 'backward')
    close_gate()