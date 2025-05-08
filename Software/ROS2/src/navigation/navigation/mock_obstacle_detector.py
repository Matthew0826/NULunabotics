"""A mock obstacle detector that generates fake obstacles in a random location."""

import rclpy
from rclpy.node import Node
from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import Obstacle, Point
from std_msgs.msg import Float32

import numpy as np
import random


# TODO: put actual values in for the offset
ROBOT_LIDAR_OFFSET = (30, 0, 20) # X, Y, Z (in cm)
                           # ^ vertical
# estimates of the real thing
LIDAR_VIEW_DISTANCE = 100 # cm
LIDAR_VIEW_SIZE = 120 # cm
# how many obstacles to generate
MOCK_OBSTACLE_COUNT = 4


def rotate_vector_2d(vector, angle_degrees):
    """Rotates a 2D vector by a given angle in degrees.

    Args:
        vector (np.ndarray): A 2D numpy array representing the vector.
        angle_degrees (float): The angle of rotation in degrees.

    Returns:
        The rotated 2D vector.
    """
    angle_radians = np.radians(angle_degrees)
    cos_theta = np.cos(angle_radians)
    sin_theta = np.sin(angle_radians)

    rotation_matrix = np.array([[cos_theta, -sin_theta],
                                [sin_theta, cos_theta]])

    rotated_vector = np.dot(rotation_matrix, vector)
    return list(rotated_vector)


class MockObstacleDetector(Node):
    """A mock obstacle detector that generates fake obstacles in a random location."""
    def __init__(self):
        super().__init__('obstacle_detector')
        self.position_subscription = self.create_subscription(
            Point,
            'sensors/position',
            self.position_callback,
            10)
        self.orientation_subscription = self.create_subscription(
            Float32,
            'sensors/orientation',
            self.orientation_callback,
            10)
        self.obstacle_publisher = self.create_publisher(Obstacle, 'navigation/obstacles', 10)
        self.robot_position = None
        self.robot_orientation = None
        self.mock_obstacles = []
        self.generate_test_obstacles()
        
    def publish_obstacle(self, x, y, radius):
        """Publish an obstacle at the given coordinates 10 times to ensure pathfinder has confidence in it."""
        for _ in range(5):  # to make condifence of this fake obstacle 100%
            ob = Obstacle()
            ob.position.x = float(x)
            ob.position.y = float(y)
            ob.radius = float(radius)
            self.obstacle_publisher.publish(ob)
    
    def generate_test_obstacles(self):
        """Generates test obstacles for the pathfinder to use.
        We know there will be rocks along the border between the obstacle zone and the dump zone."""
        #TODO: add obstacles around the edge so the robot doesn't think it can go outside of the map
        while len(self.mock_obstacles) < MOCK_OBSTACLE_COUNT:
            # pick a point anywhere in the obstacle zone
            radius = random.randint(15, 20)
            x = float(random.randint(radius, 548-radius))
            y = float(random.randint(radius, 244-radius))
            # check if obstacle would be in start zone
            if x > (348 - radius) and y < (200 + radius):
                continue
            # False means the robot hasn't seen it yet
            self.mock_obstacles.append([x, y, float(radius), False])
        # generates the line of rocks between the obstacle and construction zones
        for i in range(1,9): # loop between 1 and 8
            x = 548.0 - i*34.25 + 34.25/2
            y = 244.0
            radius = 34.25/2
            # False means the robot hasn't seen it yet
            self.mock_obstacles.append([x, y, radius, False])
        # # generate a grid of rocks to test the algorithm across the map
        # for i in range(0, 548, 20):
        #     for j in range(0, 487, 20):
        #         x = float(i)
        #         y = float(j)
        #         radius = 20
        #         # False means the robot hasn't seen it yet
        #         self.mock_obstacles.append([x, y, radius, False])
        
    # I thought of the idea for the algorithm!
    # Numpy is hard ok?
    # Courtesy of ChatGPT:
    def can_robot_see_obstacle(self, obstacle):
        """Check if the robot can see the given obstacle, accounting for real world factors such as LiDAR view distance."""
        if self.robot_position is None or self.robot_orientation is None:
            return False
        if obstacle[3]: # already seen
            return False
        robot_pos = np.array(self.robot_position)

        # Step 1: Forward-facing unit vector
        forward_vector = rotate_vector_2d(np.array([1.0, 0.0]), self.robot_orientation)
        forward_vector /= np.linalg.norm(forward_vector)

        # Step 2: LIDAR position
        lidar_offset_rotated = rotate_vector_2d(np.array([ROBOT_LIDAR_OFFSET[0], ROBOT_LIDAR_OFFSET[1]]), self.robot_orientation)
        lidar_pos = robot_pos + lidar_offset_rotated

        # Step 3: Center of the "vision screen"
        screen_center = lidar_pos + forward_vector * LIDAR_VIEW_DISTANCE

        # Step 4: Perpendicular (right-facing) vector
        perp_vector = np.array([-forward_vector[1], forward_vector[0]])

        # Step 5: Vector from screen center to obstacle
        to_obstacle = np.array([obstacle[0], obstacle[1]]) - screen_center

        # Project onto perpendicular vector to get lateral offset
        lateral_offset = np.dot(to_obstacle, perp_vector)

        # Project onto forward vector to ensure the obstacle is near the screen plane
        forward_offset = np.dot(to_obstacle, forward_vector)

        # Distance checks
        radius = obstacle[2]
        half_screen = LIDAR_VIEW_SIZE / 2
        detection_band = half_screen + radius / 2
        screen_depth_tolerance = radius / 2  # how close it must be to the plane

        if abs(lateral_offset) <= detection_band and abs(forward_offset) <= screen_depth_tolerance:
            self.get_logger().info(f"Obstacle at {obstacle} is visible")
            return True
        return False

        # Visualization guide:
        #
        # [Robot] ---> (forward_vector)
        #                 |
        #                 |  <- screen plane (centered LIDAR_VIEW_DISTANCE ahead)
        #                 |
        #         <-------|------->
        #        width = LIDAR_VIEW_SIZE
        #
        # Obstacles within this narrow rectangle (±radius/2 depth, ±(LIDAR_VIEW_SIZE/2 + radius/2) width)
        # are considered visible by the LIDAR.
    
    def check_obstacles(self):
        """Check if the robot can see any of the obstacles we invented earlier."""
        for obstacle in self.mock_obstacles:
            if self.can_robot_see_obstacle(obstacle):
                self.publish_obstacle(obstacle[0], obstacle[1], obstacle[2])
                obstacle[3] = True
    
    def position_callback(self, msg):
        """Called whenever the robot's position is updated. Updates the robot's position and check for obstacles."""
        self.robot_position = (msg.x, msg.y)
        self.check_obstacles()
    
    def orientation_callback(self, msg):
        """Called whenever the robot's orientation is updated. Updates the robot's orientation and check for obstacles."""
        self.robot_orientation = 360.0 - msg.data
        self.check_obstacles()


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(MockObstacleDetector())


if __name__ == '__main__':
    main()
