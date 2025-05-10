import rclpy
from rclpy.node import Node
import math

from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import LidarRotation, Obstacle, Point
from std_msgs.msg import Float32

import numpy as np


# TODO: put actual values in for the offset
ROBOT_LIDAR_OFFSET = (30, 0, 20) # X, Y, Z (in cm)
                           # ^ vertical
BUMP_SIZE_GUESS = 3 # degrees
BUMP_THRESHOLD = 10 # cm

# new strategy:
# - record a history of lidar rotations
# - when the accelerometer begins to move, clear the history
# - 

class MovingAverage:
    """A simple moving average class to keep track of the last N LiDAR rotations."""

    def __init__(self, size):
        self.size = size
        self.data = [[] for _ in range(360)]  # 360 degrees
        self.count = 0

    def add(self, points):
        for point in points:
            self.data[int(point.angle) % 360].append(point.distance)
            self.data[int(point.angle) % 360] = self.data[int(point.angle)][-self.size:]
        self.count += 1
        if self.count > self.size:
            self.count = self.size

    def get_average(self):
        averages = []
        for distances in self.data:
            if distances:
                averages.append(sum(distances) / len(distances))
            else:
                averages.append(0)
        return averages

    
    def get_average_points(self):
        avg_distances = self.get_average()
        return [(math.cos(math.radians(i)) * avg_distances[i], math.sin(math.radians(i)) * avg_distances[i]) for i in range(360)]
    
    def is_full(self):
        return self.count >= self.size
    
    def clear(self):
        self.data = [[] for _ in range(360)]
        self.count = 0


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
        self.orientation_subscription = self.create_subscription(
            Float32,
            'sensors/orientation',
            self.orientation_callback,
            10)
        self.obstacle_publisher = self.create_publisher(Obstacle, 'navigation/obstacles', 10)
        self.robot_position = None
        self.robot_orientation = None
        self.recent_average = MovingAverage(10)
        self.long_term_average = MovingAverage(200)
        
    def publish_obstacle(self, x, y, radius):
        if self.robot_position is None or self.robot_orientation is None:
            return
        obstacle = Obstacle()
        position = Point()
        # adjust x and y based on robot orientation
        # x, y = rotate_vector_2d(np.array([x + ROBOT_LIDAR_OFFSET[0], y + ROBOT_LIDAR_OFFSET[1]]), self.robot_orientation)
        # world_x = self.robot_position[0] + x
        # world_y = self.robot_position[1] + y
        position.x = x#world_x
        position.y = y#world_y
        obstacle.position = position
        obstacle.radius = radius    
        self.obstacle_publisher.publish(obstacle)

    def lidar_callback(self, msg):
        self.recent_average.add(msg.points)
        self.long_term_average.add(msg.points)
        # wait until we have enough data to process
        if not self.recent_average.is_full():
            return
        recent_avg_distances = self.recent_average.get_average()
        long_term_avg_distances = self.long_term_average.get_average()
        # calculate the difference between the recent and long term averages (in cm)
        differences = [recent - long_term for recent, long_term in zip(recent_avg_distances, long_term_avg_distances)]
        # find bumps in the data (the bumps represent rocks or craters)
        bump_centers = []
        for i in range(0, 360, BUMP_SIZE_GUESS):
            window = differences[i:i + BUMP_SIZE_GUESS]
            if len(window) < BUMP_SIZE_GUESS:
                break
            avg_window = sum([abs(p) for p in window]) / len(window)
            if avg_window > BUMP_THRESHOLD:
                bump_centers.append(i + BUMP_SIZE_GUESS // 2)
        # find how big each bump is by gradually increasing the window size until it covers the entire bump
        bumps = []
        for degree in bump_centers:
            lower = degree - BUMP_SIZE_GUESS // 2
            upper = degree + BUMP_SIZE_GUESS // 2
            window = differences[lower:upper]
            while all([abs(p) > BUMP_THRESHOLD for p in window]):
                lower -= 1
                upper += 1
                if upper > 360: upper = 0
                if lower < 0:   lower = 359
                window = differences[lower:upper]
            bumps.append((lower, upper))
        # filter out bumps that are inside another bump
        filtered_bumps = []
        for lower, upper in bumps:
            is_duplicate = False
            for other_lower, other_upper in bumps:
                does_overlap = (lower <= other_lower < upper) or (lower < other_upper <= upper)
                if does_overlap:
                    is_duplicate = True
                    break
            if not is_duplicate:
                filtered_bumps.append((lower, upper))
        # publish bumps as obstacles
        for lower, upper in filtered_bumps:
            # find average distance in the bump
            size = 0
            for degree in range(lower, upper):
                size += recent_avg_distances[degree]
            size /= (upper - lower)
            x = math.cos(math.radians(degree)) * recent_avg_distances[degree]
            y = math.sin(math.radians(degree)) * recent_avg_distances[degree]
            self.publish_obstacle(x, y, size)
        self.recent_average.clear()
        

    
    def position_callback(self, msg):
        self.robot_position = (msg.x, msg.y)
    
    def orientation_callback(self, msg):
        self.robot_orientation = msg.data


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(ObstacleDetector())
    


if __name__ == '__main__':
    main()
