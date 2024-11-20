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
    

# Main loop
def main():
    
    try:
        Grabbing.collect_block()
        
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Stopping and resetting")
        reset_brick()
        exit()

if __name__ == "__main__":
    main()
