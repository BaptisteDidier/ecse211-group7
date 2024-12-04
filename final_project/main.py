from Resources import *
import Motion
import Grabbing

initialize_components()

# Main loop
def main():
    try:
        Motion.move(40, 8, 'forward')
        Motion.orient_and_pickup()
        Motion.move(40, 15, 'forward')
        Grabbing.open_gate()
        Motion.move(40, 15, 'backward')
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
