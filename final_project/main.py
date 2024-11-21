from Resources import *
from threading import Thread
import Motion
import Grabbing

initialize_components()

# Public methods
def run_in_background(action):
    """
    Runs the given function in the background of the main thread
    """
    Thread(target=action, daemon=True).start()
    
def go_to_block():
    temp_list = []
    run_in_background(Motion.turn(30, 90))
    if ultrasonic_sensor.get_cm() < 110 and ultrasonic_sensor.get_cm > 0:
        distance = ultrasonic_sensor.get_cm()
        left_motor_val = Motion.left_motor.get_encoder()
        right_motor_val = Motion.right_motor.get_encoder()
        temp_list.append((distance, left_motor_val, right_motor_val))
    
    smallest_tuple = min(temp_list, key=lambda x: x[0])
    cube_x_pos = Motion.x + ultrasonic_sensor.get_cm()*math.cos(Motion.theta)
    cube_y_pos = Motion.y + ultrasonic_sensor.get_cm()*math.sin(Motion.theta)
    Motion.move_to(cube_x_pos, cube_y_pos)
        

    
# Main loop
def main():
    
    try:
        #run_in_background(Motion.odometry())
        Motion.move(50, 20)
        Motion.turn(50, 90)
        Motion.move(50, 10)
        #print(Motion.get_position())
        #Grabbing.collect_block()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
