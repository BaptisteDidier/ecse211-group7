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
    gate_motor.set_position_relative(65)
        
def close_gate():
    """
    Close the gate
    """
    gate_motor.set_position_relative(-65)
    
def collect():
    """
    Collect a cube in front of the robot
    """
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
    #collected_cubes = 0

def collect():
    print("collect")
    move(40, 3, 'backward')
    open_gate()
    move(40, 10, 'forward')
    close_gate()
    move(40, 7, 'backward')