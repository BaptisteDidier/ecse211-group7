from Resources import *
import Motion
import ColorDetection
import Grabbing

initialize_components()

def main():
    
    try:
        Motion.calibrate()
        Motion.move(50, 20, 'forward')
        
        ColorDetection.get_normalized_rgb()
        
        Grabbing.open()
        time.sleep(2)
        Grabbing.close()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        reset_brick()

if __name__ == "__main__":
    main()
