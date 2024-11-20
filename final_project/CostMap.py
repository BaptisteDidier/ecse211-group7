from Resources import ultrasonic_sensor
import Motion
import math

# Global variable
obstacles = list()
costmap = [[0 for _ in range(122)] for _ in range(122)]

def get_obstacle_position():
    """
    Get the position of the obstacle in front depending on the current position
    """
    distance = ultrasonic_sensor.get_cm()
    if distance is None:
        return None;
    
    x, y, theta = Motion.get_position()
    new_x = x + distance*math.cos(theta)
    new_y = y + distance*math.sin(theta)
    
    if not (0 < new_x < 122) or not (0 < new_y < 122): # if the sensor sees the walls
        return None
    
    return new_x, new_y

def add_obstacle(obstacle_position):
    """
    Add an obstacle to the costmap
    """
    if obstacle_position is None or any(value < 0 or value >= 122 for value in obstacle_position):
        print("Invalid obstacle")
        return;
 
    obstacles.append(obstacle_position)
    
def scan():
    """
    Turns to assess where the obstacles are
    """
    Motion.turn(50, 180, "right")
        