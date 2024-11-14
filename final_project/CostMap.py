class CostMap:
    
    def __init__(self):
        self.map_width = 122
        self.map_height = 122
        self.cube_size = 2.54
        self.width = self.map_width // self.cube_size
        self.height = self.map_height // self.cube_size
        self.costmap = [[0 for _ in range(self.width)] for _ in range(self.height)]
        
    def __str__(self):
        """
        Display the cost map
        """
        return "\n".join(" ".join(f"{cell:3d}" for cell in row) for row in self.costmap)

    def add_obstacle(self, obstacle_position):
        """
        Add an obstacle to the costmap
        """
        x, y = obstacle_position
        cell_x = int(x // self.cube_size)
        cell_y = int(y // self.cube_size)
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            self.costmap[cell_y][cell_x] = 1
    
    def add_water(self, water_position):
        x, y = water_position
        cell_x = int(x // self.cube_size)
        cell_y = int(y // self.cube_size)
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            self.costmap[cell_y][cell_x] = float("inf")
    
    def get_cost(self, point):
        """
        Get the cost of a point in the costmap.
        Converts real-world coordinates to grid coordinates and checks if the point is within bounds.
        """
        x, y = point
        cell_x = int(x // self.cube_size)
        cell_y = int(y // self.cube_size)
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            return self.costmap[cell_y][cell_x]
        return float("inf")
