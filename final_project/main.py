from Resources import *
import Motion
from WallCheck import check_water
initialize_components()

# Public methods   

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
        onEdge = 1
        Motion.move(40, 14)
        Motion.turn(40, -90, 'left')
        onEdge = 0
        
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
