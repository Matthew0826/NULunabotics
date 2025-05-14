from navigation.astar import AStar

# these are rounded up to the nearest 5 cm
MAP_WIDTH = 550  # units: cm
MAP_HEIGHT = 490
GRID_RESOLUTION = 2  # units: cm
GRID_WIDTH = MAP_WIDTH // GRID_RESOLUTION
GRID_HEIGHT = MAP_HEIGHT // GRID_RESOLUTION

# enum for zones on the map
OUT_OF_BOUNDS = -1
START_ZONE = 0
TRAVERSAL_ZONE = 1
EXCAVATION_ZONE = 2
DUMP_ZONE = 3
BERM_ZONE = 4


def world_to_grid(x, y):
    """Converts world coordinates (cm) to grid coordinates (GRID_RESOLUTION x GRID_RESOLUTION cm squares)."""
    return int(x // GRID_RESOLUTION), int(y // GRID_RESOLUTION)


def grid_to_world(x, y):
    """
    Converts grid coordinates (GRID_RESOLUTION x GRID_RESOLUTION cm squares) to world coordinates (cm).
    Places it in the center of the grid square.
    """
    return x * GRID_RESOLUTION + GRID_RESOLUTION/2, y * GRID_RESOLUTION + GRID_RESOLUTION/2


def get_zone(x, y):
    """Returns the zone of the map that the given coordinates are in (world coordinates).
    Options are START_ZONE, TRAVERSAL_ZONE, EXCAVATION_ZONE, DUMP_ZONE, BERM_ZONE, OUT_OF_BOUNDS."""
    if x < 0 or x >= MAP_WIDTH or y < 0 or y >= MAP_HEIGHT:
        return OUT_OF_BOUNDS
    if y < 200:
        if x > MAP_WIDTH - 200:
            return START_ZONE
        return BERM_ZONE
    if x < 274:
        return EXCAVATION_ZONE
    return DUMP_ZONE


def distance(point1, point2) -> float:
    """Returns the distance between two points."""
    return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5


class AStarNode:
    """Represents a 5x5 cm square on the map (node on the graph). x and y are coordinates in cm on the grid (not world)."""
    def __init__(self, x, y, confidence=0):
        self.x = x
        self.y = y
        self.confidence = confidence
    
    @classmethod
    def from_point(cls, point):
        world_x, world_y = world_to_grid(point.x, point.y)
        return cls(world_x, world_y, 0)
        
    def distance(self, other):
        return distance(self, other)
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __str__(self):
        return f"AStarNode({self.x}, {self.y}, confidence={self.confidence})"


# TODO: add a better heuristic cost function to the A* algorithm to make it faster? maybe not needed
class LunaboticsAStar(AStar):
    """Used for the astar package to find a path through the grid of nodes for the robot to drive."""
    def __init__(self, nodes, confidence_threshold):
        self.nodes = nodes
        self.confidence_threshold = confidence_threshold

    def neighbors(self, n):
        # TODO: account for robot width and length when finding neighbors (not just 8!)
        # loop through all 8 neighbors of the node
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                x = n.x + i
                y = n.y + j
                if x < 0 or x >= GRID_WIDTH or y < 0 or y >= GRID_HEIGHT:
                    continue
                node = self.nodes[int(y)][int(x)]
                is_obstacle = node.confidence >= self.confidence_threshold
                world_x, world_y = grid_to_world(x, y)
                if get_zone(world_x, world_y) != OUT_OF_BOUNDS and not is_obstacle:
                    yield node

    def distance_between(self, n1, n2):
        return n1.distance(n2)
            
    def heuristic_cost_estimate(self, current, goal):
        return current.distance(goal)
    
    def is_goal_reached(self, current, goal):
        return current == goal
