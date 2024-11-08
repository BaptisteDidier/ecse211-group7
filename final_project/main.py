from Motion import Motion
from utils.brick import wait_ready_sensors

motion = Motion()

wait_ready_sensors(True)
print("Done initializing")

def main():
    
    try:
        motion.calibrate()
        motion.move_forward(50, 2)
        motion.turn_right(50, 2)
        motion.turn_left(50, 2)
        motion.move_forward(50, 2)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        motion.reset()

if __name__ == "__main__":
    main()
