import heapq

class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

class PathFinding:
    def __init__(self, cost_map):
        self.cost_map = cost_map
    
    def find_path(self, start, goal):
        open_list = []
        closed_set = set()
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])
        
        heapq.heappush(open_list, (start_node.f, start_node))
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            if (current.x, current.y) == (goal_node.x, goal_node.y):
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1]  # Return reversed path
            
            closed_set.add((current.x, current.y))
            
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Move in 4 directions
                nx, ny = current.x + dx, current.y + dy
                if (nx, ny) in closed_set or not self.cost_map.is_walkable(nx, ny):
                    continue
                
                neighbor = Node(nx, ny, current.g + self.cost_map.get_cost(nx, ny), heuristic(Node(nx, ny), goal_node), current)
                
                heapq.heappush(open_list, (neighbor.f, neighbor))
        
        return None  # No path found
