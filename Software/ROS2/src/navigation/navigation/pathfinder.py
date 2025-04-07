import rclpy
from rclpy.node import Node

from lunabotics_interfaces.msg import Point, Obstacle
from lunabotics_interfaces.srv import Path

from astar import AStar
import random

# Note: 0, 0 is defined as the top left corner of the map

# these are rounded up to the nearest 5 cm
MAP_WIDTH = 550  # units: cm
MAP_HEIGHT = 490
GRID_RESOLUTION = 5  # units: cm
GRID_WIDTH = MAP_WIDTH // GRID_RESOLUTION
GRID_HEIGHT = MAP_HEIGHT // GRID_RESOLUTION
# TODO: use actual robot dimensions
ROBOT_WIDTH = 30  # units: cm
ROBOT_LENGTH = 50  # units: cm

# enum for zones on the map
OUT_OF_BOUNDS = -1
START_ZONE = 0
TRAVERSAL_ZONE = 1
EXCAVATION_ZONE = 2
DUMP_ZONE = 3
BERM_ZONE = 4

# threshold needed for a cell to be considered an obstacle between 0 and 1
INITIAL_OBSTACLE_CONFIDENCE_THRESHOLD = 0.2
HIGHEST_CONFIDENCE_THRESHOLD = 0.7
OBSTACLE_SIZE_THRESHOLD = 10  # units: cm
# amount that each obstacle detection adds to the grid cells
OBSTACLE_CONFIDENCE_STRENGTH = 0.12


def world_to_grid(x, y):
    return x // GRID_RESOLUTION, y // GRID_RESOLUTION


def grid_to_world(x, y):
    return x * GRID_RESOLUTION + GRID_RESOLUTION/2, y * GRID_RESOLUTION + GRID_RESOLUTION/2


def get_zone(x, y):
    if x < 0 or x > MAP_WIDTH or y < 0 or y > MAP_HEIGHT:
        return OUT_OF_BOUNDS
    if y < 200:
        if x > MAP_WIDTH - 200:
            return START_ZONE
        return BERM_ZONE
    if x < 274:
        return EXCAVATION_ZONE
    return DUMP_ZONE


class Node:
    def __init__(self, x, y, confidence=0):
        self.x = x
        self.y = y
        self.confidence = confidence
    
    @classmethod
    def from_point(cls, point):
        return cls(point.x, point.y, 0)
        
    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __str__(self):
        return f"Node({self.x}, {self.y}, confidence={self.confidence})"


# TODO: add a better heuristic cost function to the A* algorithm to make it faster? maybe not needed
class LunaboticsAStar(AStar):
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
                node = self.nodes[y][x]
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


class Pathfinder(Node):

    def __init__(self):
        super().__init__('pathfinder')
        # these are any obstacles detected by the lidar system
        self.subscription = self.create_subscription(
            Obstacle,
            'navigation/obstacles',
            self.obstacle_callback,
            10)
        self.subscription  # prevent unused variable warning
        # requests are start and end points for the pathfinder,
        # and it returns a list of points representing the path
        # Note: may return empty list meaning no path was found!
        self.srv = self.create_service(Path, 'pathfinder', self.path_service_callback)
        # represents the map as a grid of GRID_RESOLUTION cm squares
        # each square is a float from 0 to 1 representing the probability of that square being an obstacle
        # 0 = no obstacle, 1 = obstacle
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        self.generate_test_obstacles()

    def calculate_path(self, start, end, confidence_threshold):
        start_node = Node.from_point(start)
        goal_node = Node.from_point(end)
        # create a grid of nodes with the confidence values
        nodes = [[Node(x, y, confidence=self.grid[y][x]) for x in range(GRID_WIDTH)] for y in range(GRID_HEIGHT)]
        a_star = LunaboticsAStar(nodes, confidence_threshold)
        path = a_star.find_path(start_node, goal_node)
        if path is not None:
            return path
        return []

    def add_confidence(self, world_x, world_y, confidence):
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        if get_zone(world_x, world_y) != OUT_OF_BOUNDS:
            # max confidence is 1
            self.grid[grid_y][grid_x] = min(1, self.grid[grid_y][grid_x] + confidence)
    
    def generate_test_obstacles(self):
        # generate test obstacles in the grid
        for i in range(4):
            obstacle = Obstacle()
            obstacle.position.x = random.randint(0, MAP_WIDTH)
            obstacle.position.y = random.randint(0, MAP_HEIGHT)
            obstacle.radius = random.randint(30, 40)
            self.obstacle_callback(obstacle)

    def obstacle_callback(self, msg):
        world_x = msg.position.x
        world_y = msg.position.y
        radius = msg.radius
        if radius < OBSTACLE_SIZE_THRESHOLD: return
        # find all cm^2 that are within the radius of the obstacle
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                # only use points inside of the circle
                if i ** 2 + j ** 2 <= radius ** 2:
                    # add a small bit of confidence such that squares fully within the radius are more likely to be an obstacle
                    # grid cells only partially covered by the circle are less likely to be an obstacle
                    # my thinking is that as the robot tries to find an obstacle many times, it will think its a few centimeters one way or the other each time, so this averages that out
                    self.add_confidence(world_x + i, world_y + j, OBSTACLE_CONFIDENCE_STRENGTH/(GRID_RESOLUTION**2))
        
    def path_service_callback(self, request, response):
        start = request.start
        end = request.end
        # search for a path until one is found or the confidence threshold is too low
        path = []
        confidence_threshold = INITIAL_OBSTACLE_CONFIDENCE_THRESHOLD
        while len(path) == 0 and confidence_threshold <= HIGHEST_CONFIDENCE_THRESHOLD:
            path = self.calculate_path(start, end, confidence_threshold)
            confidence_threshold += OBSTACLE_CONFIDENCE_STRENGTH
        # get path in world points
        path = [self.grid_to_world(node.x, node.y) for node in path]  # convert to world coordinates
        response.nodes = [Point(x=point[0],y=point[1]) for point in path]
        return response


def main(args=None):
    rclpy.init(args=args)
    pathfinder = Pathfinder()
    rclpy.spin(pathfinder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
