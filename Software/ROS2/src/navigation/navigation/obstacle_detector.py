import rclpy
from rclpy.node import Node

from navigation.obstacles import MovingAverage, PointProcessor, FeatureDetector
from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import LidarRotation, Obstacle, Point, Acceleration

import numpy as np


# TODO: put actual values in for the offset
ROBOT_LIDAR_OFFSET = (37, 0, 19) # X, Y, Z (in cm)
                           # ^ vertical

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

ACCELERATION_THRESHOLD = 0.7 # m/s^2

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.lidar_subscription = self.create_subscription(
            LidarRotation,
            'sensors/lidar',
            self.lidar_callback,
            10)
        self.position_subscription = self.create_subscription(
            Point,
            'sensors/position',
            self.position_callback,
            10)
        self.acceleration_subscription = self.create_subscription(
            Acceleration,
            'sensors/acceleration',
            self.acceleration_callback,
            10)
        self.obstacle_publisher = self.create_publisher(Obstacle, 'navigation/obstacles', 10)
        self.robot_position = (448, 100)
        self.robot_orientation = 0.0
        self.acceleration = (0, 0)
        self.average = MovingAverage(10)
        self.point_processor = PointProcessor(self.get_logger())
        self.feature_detector = FeatureDetector(self.get_logger())
        # self.long_term_average = MovingAverage(200)
        
    def publish_obstacle(self, x, y, radius, is_rock):
        obstacle = Obstacle()
        position = Point()
        relative_position = Point()
        relative_position.x = x
        relative_position.y = y
        # adjust x and y based on robot orientation
        if self.robot_position is not None and self.robot_orientation is not None:
            x, y = rotate_vector_2d(np.array([x + ROBOT_LIDAR_OFFSET[0], y + ROBOT_LIDAR_OFFSET[2]]), self.robot_orientation - 90)
            x = self.robot_position[0] + x
            y = self.robot_position[1] + y
        position.x = float(x)
        position.y = float(y)
        obstacle.position = position
        obstacle.radius = float(radius)
        obstacle.is_rock = is_rock
        obstacle.relative_position = relative_position
        self.obstacle_publisher.publish(obstacle)
    
    def is_robot_moving(self):
        # Check if the robot is moving based on acceleration
        x_accel, y_accel = self.acceleration
        if abs(x_accel) > ACCELERATION_THRESHOLD or abs(y_accel) > ACCELERATION_THRESHOLD:
            return True

    def lidar_callback(self, msg):
        if self.is_robot_moving():
            # if the robot is moving, clear the moving average
            # the lidar will not work properly if the robot is moving
            self.average.clear()
            return
        self.average.add(msg.points)
        # wait until we have enough data to process
        if not self.average.is_full():
            return
        # calculate average
        average_points = self.average.get_average_points()
        # take out outliers
        points = self.point_processor.preprocess(np.array(average_points))
        # use curvature to find features (rocks or craters)
        features = self.feature_detector.detect_features(points)
        for feature in features:
            self.publish_obstacle(*feature)
        self.average.clear()
    
    def position_callback(self, msg):
        self.robot_position = (msg.x, msg.y)
    
    def acceleration_callback(self, msg: Acceleration):
        self.robot_orientation = msg.orientation
        self.acceleration = (msg.acceleration_x, msg.acceleration_y)


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(ObstacleDetector())
    


if __name__ == '__main__':
    main()
