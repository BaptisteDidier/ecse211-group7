from threading import Thread

def run_in_background(action):
    """
    Runs the given function in the background of the main thread
    """
    Thread(target=action, daemon=True).start()