import rclpy
from rclpy.node import Node

from lunabotics_interfaces.msg import Point, Obstacle, PathVisual
from lunabotics_interfaces.srv import Path

from navigation.pathfinder_helper import *
import random
import math

# Note: 0, 0 is defined as the top left corner of the map

# TODO: make this a ros parameter
ROBOT_WIDTH = 71 # units: cm
ROBOT_LENGTH = 98  # units: cm

# threshold needed for a cell to be considered an obstacle between 0 and 1
INITIAL_OBSTACLE_CONFIDENCE_THRESHOLD = 0.2
HIGHEST_CONFIDENCE_THRESHOLD = 0.7
OBSTACLE_SIZE_THRESHOLD = 10  # units: cm
# amount that each obstacle detection adds to the grid cells
OBSTACLE_CONFIDENCE_STRENGTH = 0.12




class Pathfinder(Node):
    """
    The pathfinder node that finds a path through the map using the A* algorithm.
    Uses a confidence map to determine if a square is an obstacle or not.
    1 = obstacle, 0 = no obstacle.
    """
    
    def __init__(self):
        super().__init__('pathfinder')
        # these are any obstacles detected by the lidar system
        self.subscription = self.create_subscription(
            Obstacle,
            'navigation/obstacles',
            self.obstacle_callback,
            10)
        # requests are start and end points for the pathfinder,
        # and it returns a list of points representing the path
        # Note: may return empty list meaning no path was found!
        self.srv = self.create_service(Path, 'pathfinder', self.path_service_callback)
        # represents the map as a grid of GRID_RESOLUTION cm squares
        # each square is a float from 0 to 1 representing the probability of that square being an obstacle
        # 0 = no obstacle, 1 = obstacle
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        
        self.website_path_visualizer = self.create_publisher(PathVisual, 'navigation/path', 10)

    def calculate_path(self, start, end, confidence_threshold):
        """Finds a path from start to end using the A* algorithm."""
        start_node = AStarNode.from_point(start)
        goal_node = AStarNode.from_point(end)
        # create a grid of nodes with the confidence values
        nodes = [[AStarNode(x, y, confidence=self.grid[y][x]) for x in range(GRID_WIDTH)] for y in range(GRID_HEIGHT)]
        a_star = LunaboticsAStar(nodes, confidence_threshold)
        path = a_star.astar(start_node, goal_node)
        if path is not None:
            return list(path)
        return []

    def add_confidence(self, world_x, world_y, confidence):
        """Adds confidence to the grid at the given world coordinates in cm."""
        grid_x, grid_y = world_to_grid(world_x, world_y)
        if get_zone(world_x, world_y) != OUT_OF_BOUNDS:
            # max confidence is 1
            self.grid[grid_y][grid_x] = min(1, self.grid[grid_y][grid_x] + confidence)

    def obstacle_callback(self, msg):
        """Treats rocks and craters as circles and adds them to the grid.
        Squares that are only partially covered by the circle are less likely to be an obstacle."""
        world_x = msg.position.x
        world_y = msg.position.y
        radius = int(math.ceil(msg.radius))
        if radius < OBSTACLE_SIZE_THRESHOLD: return
        radius += ROBOT_WIDTH // 2 + 5 # add a bit of padding to the obstacle size to account for robot width
        # find all cm^2 that are within the radius of the obstacle
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                # only use points inside of the circle
                if i ** 2 + j ** 2 <= radius ** 2:
                    # add a small bit of confidence such that squares fully within the radius are more likely to be an obstacle
                    # grid cells only partially covered by the circle are less likely to be an obstacle
                    # my thinking is that as the robot tries to find an obstacle many times,
                    # it will think its a few centimeters one way or the other each time, so this averages that out
                    self.add_confidence(world_x + i, world_y + j, OBSTACLE_CONFIDENCE_STRENGTH/(GRID_RESOLUTION**2))
        
    def filter_nearby_points(self, path, min_distance):
        """Filters out points that are too close to each other in the path.
        This is to decrease the amount of steps the odometry node has to complete, as well as fixing a rendering bug on the website."""
        filtered_path = []
        for i in range(len(path) - 1):
            for j in range(len(filtered_path)):
                if path[i].distance(filtered_path[j]) <= min_distance:
                    break
            else: # if it didn't break, add the point to the filtered path
                filtered_path.append(path[i])
        if len(filtered_path) > 0:  # otherwise the last point is excluded
            filtered_path.pop()  # to stop the last 2 points from being too close
            filtered_path.append(path[-1])
        return filtered_path
    
    def path_service_callback(self, request, response):
        """
        This function is called whenever a path is requested.
        It finds a path from start to end, beginning with using all obstacles of any confidence threashold,
        before moving on to only using high confidence obstacles to reach the destination.
        """
        start = request.start
        end = request.end
        # search for a path until one is found or the confidence threshold is too low
        path = []
        confidence_threshold = INITIAL_OBSTACLE_CONFIDENCE_THRESHOLD
        while len(path) == 0 and confidence_threshold <= HIGHEST_CONFIDENCE_THRESHOLD:
            path = self.calculate_path(start, end, confidence_threshold)
            if len(path) == 0:
                confidence_threshold += OBSTACLE_CONFIDENCE_STRENGTH
        # filter out points that are 7 cm away from each other
        filtered_path = self.filter_nearby_points(path, 7)
        # get path in world points
        filtered_path = [grid_to_world(node.x, node.y) for node in filtered_path]
        response.nodes = [Point(x=point[0],y=point[1]) for point in filtered_path]
        print("Path found with confidence threshold:", confidence_threshold)
        print("Start:", start.x, ",", start.y, "  End:", end.x, ",", end.y)
        # self.debug_map()
        self.website_path_visualizer.publish(PathVisual(nodes=response.nodes))
        return response
    
    def debug_map(self):
        """Print entire map for debugging using ascii art to represent the confidence values
        options from least filled in to most filled in."""
        ascii_options = [" ", ".", "*", ":", "o", "8", "X", "#", "@", "%"]
        for i in range(GRID_HEIGHT):
            for j in range(GRID_WIDTH):
                confidence = self.grid[i][j]
                print(ascii_options[min(9, int(confidence * 10))], end="")
            print()


def main(args=None):
    rclpy.init(args=args)
    pathfinder = Pathfinder()
    print("Pathfinder is running!")
    rclpy.spin(pathfinder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
