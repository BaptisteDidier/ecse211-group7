from Resources import *
import Motion

initialize_components()

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
        
        #check_water()   
        #Motion.turn(40, 90)
        #onEdge = 1
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        #onEdge = 0
        #check_water()   
        #Motion.turn(40, 90)
        #onEdge = 1
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        #onEdge = 0
        
        #check_water()   
        #Motion.turn(40, 90)
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        #check_water()   
        #Motion.turn(40, 90)
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        
        #check_water()   
        #Motion.turn(40, 90)
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        #check_water()   
        #Motion.turn(40, 90)
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        
        #check_water()
        #Motion.turn(40, 90)
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        #check_water()
        #Motion.turn(40, 90)
        #Motion.move(40, 14)
        #Motion.turn(40, -90, 'left')
        
        #check_water() 
        #check_water() 
        
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
        #print(block_color_sensor.get_rgb())
        #Motion.orient_and_pickup()
        #Motion.detect_cubes()
        #Motion.orient_and_pickup()
        Motion.thread_turn(30, 90, 'right')
        #Motion.change_row()
        #Motion.move(40, 1, 'backward')
        #print(sweep)
        #print("3")
        #Motion.sweep(-45, 45)
        #print("2")
        #Motion.thread_sweep()
        #Motion.move()
        #print(block_color_sensor.get_rgb())
        
        
        #Grabbing.
        #threads = threading.enumerate()
        #for thread in threads:
        #    print(thread.name)
        #print(ultrasonic_sensor.get_cm())
        
    except KeyboardInterrupt:
        pass#
    
    finally:
        #sweeping_motor.set_position_relative(0)
        #sweeping_motor.set_position_relative(0)
        print("Stopping and resetting")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
