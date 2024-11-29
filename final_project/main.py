from Resources import *
import threading
import time
import Motion

initialize_components()

# Main loop
def main():
    try:
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
        
        #threads = threading.enumerate()
        #for thread in threads:
        #    print(thread.name)
        #print(ultrasonic_sensor.get_cm())
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
