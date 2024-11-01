class CostMap:
    
    def __init__(self):
        self.map_width = 122
        self.map_height = 122
        self.cube_size = 2.54
        self.width = self.map_width // self.cube_size
        self.height = self.map_height // self.cube_size
        self.costmap = [[0 for _ in range(self.width)] for _ in range(self.height)]
        
    def __str__(self):
        """Display the cost map"""
        return "\n".join(" ".join(f"{cell:3d}" for cell in row) for row in self.costmap)

    def add_obstacle(self, obstacle_position):
        """Add a cube obstacle to the costmap."""
        x, y = obstacle_position
        
        cell_x = x // self.cube_size
        cell_y = y // self.cube_size

        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            self.costmap[cell_y][cell_x] = 100


