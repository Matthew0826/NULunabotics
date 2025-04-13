import rclpy
from rclpy.node import Node
import math

from lunabotics_interfaces.msg import LidarRotation, Obstacle, Point
from std_msgs.msg import Float32

import numpy as np
import random


# TODO: put actual values in for the offset
ROBOT_LIDAR_OFFSET = (30, 0, 20) # X, Y, Z (in cm)
                           # ^ vertical
LIDAR_VIEW_DISTANCE = 100 # cm
LIDAR_VIEW_SIZE = 120 # cm
BUMP_SIZE_GUESS = 3 # degrees
BUMP_THRESHOLD = 10 # cm


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
        while len(self.mock_obstacles) < 6:
            # pick a point anywhere in the obstacle zone
            x = float(random.randint(0, 548))
            y = float(random.randint(0, 305))
            radius = float(random.randint(20, 30))
            # check if obstacle would be in start zone
            if x > (348 - radius) and y < (200 + radius):
                continue
            self.mock_obstacles.append((x, y, radius))
        
    def publish_obstacle(self, x, y, radius):
        if self.robot_position is None or self.robot_orientation is None:
            return
        obstacle = Obstacle()
        position = Point()
        # adjust x and y based on robot orientation
        x, y = rotate_vector_2d(np.array([x + ROBOT_LIDAR_OFFSET[0], y + ROBOT_LIDAR_OFFSET[1]]), self.robot_orientation)
        world_x = self.robot_position[0] + x
        world_y = self.robot_position[1] + y
        position.x = world_x
        position.y = world_y
        obstacle.position = position
        obstacle.radius = radius    
        self.obstacle_publisher.publish(obstacle)
        
    # Courtesy of ChatGPT:
    def can_robot_see_obstacle(self, obstacle):
        if self.robot_position is None or self.robot_orientation is None:
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
        for obstacle in self.mock_obstacles:
            if self.can_robot_see_obstacle(obstacle):
                ob = Obstacle()
                ob.position.x = float(obstacle[0])
                ob.position.y = float(obstacle[1])
                ob.radius = float(obstacle[2])
                self.obstacle_publisher.publish(ob)
    
    def position_callback(self, msg):
        self.robot_position = (msg.x, msg.y)
        self.check_obstacles()
    
    def orientation_callback(self, msg):
        self.robot_orientation = msg.data
        self.check_obstacles()


def main(args=None):
    rclpy.init(args=args)

    mock_obstacle_detector = MockObstacleDetector()
    rclpy.spin(mock_obstacle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mock_obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
