from Resources import *
import Motion
import Grabbing
from Multithread import run_in_background
from WallCheck import check_water, is_water

initialize_components()

# Public methods   
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
        #Motion.move(40, 50)
        #Motion.turn(50, 90)
        #Motion.move(50, 70)
        #print(Motion.get_position())
        #Grabbing.collect_block()
        #Motion.explore()
        
        check_water()   
        Motion.turn(40, 90)
        Motion.move(40, 14)
        Motion.turn(40, -90, 'left')
        
        check_water()   
        Motion.turn(40, 90)
        Motion.move(40, 14)
        Motion.turn(40, -90, 'left')
        
        check_water()   
        Motion.turn(40, 90)
        Motion.move(40, 14)
        Motion.turn(40, -90, 'left')
        
        check_water()
        Motion.turn(40, 90)
        Motion.move(40, 14)
        Motion.turn(40, -90, 'left')
        
        check_water() 
        
            #else:
            #    Motion.turn(40, 45, 'right')
            #    Motion.move(40, 5)
            #    Motion.turn(40, -10, 'left')
            #    Motion.move(40, 5, 'backward')
            #    Motion.turn(40, -10, 'left')
            #    Motion.move(40, 5, 'backward')
            #    Motion.turn(40, -10, 'left')
            #    Motion.move(40, 5, 'backward')
            #    Motion.turn(40, -10, 'left')
           
        #Motion.turn(40, -90, 'left')
        #Motion.turn(40, 90)
        #print(is_water())
        #check_water()
        print(block_color_sensor.get_rgb())
        
        
    except KeyboardInterrupt:
        pass#
    
    finally:
        print("Stopping and resetting")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
