import brickpi3
from Motion import Motion
from utils.brick import wait_ready_sensors

BP = brickpi3.BrickPi3()
MOTOR1 = BP.PORT_A
MOTOR2 = BP.PORT_B

wait_ready_sensors(True)
print("Done initializing")

def main():
    motion = Motion(left_motor_port=MOTOR1, right_motor_port=MOTOR2)
    
    try:
        motion.move_forward(50, 2)
        motion.turn_left(30, 1)
        motion.move_forward(50, 2)
        motion.turn_right(30, 1)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        motion.stop()
        motion.reset()

if __name__ == "__main__":
    main()
