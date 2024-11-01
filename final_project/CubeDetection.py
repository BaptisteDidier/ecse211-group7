from utils.brick import EV3UltrasonicSensor

class CubeDetection:
    cube_threshold = 15
    wall_threshold = 30
    
    def __init__(self, ultrasonic_sensor_port):
        self.ultrasonic_sensor = EV3UltrasonicSensor(ultrasonic_sensor_port)
        self.cubes = []
        self.robot_position = (0, 0)

    def get_distance(self):
        """Return the distance from the ultrasonic sensor."""
        return self.ultrasonic_sensor.get_raw_value()
    
    def get_cubes(self):
        """Return the list of detected cubes."""
        return self.cubes
    
    def get_position(self):
        """Return the current position of the robot."""
        return self.robot_position

    # We need to find a way to distinguish cubes from the walls
    def detect_objects(self):
        """Check for cubes in the current direction."""
        forward_distance = self.get_distance()

        if forward_distance < CubeDetection.cube_threshold:
            self.cubes.append(self.get_position())
