import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from lunabotics_interfaces.msg import LidarRotation, LiDAR
from sensor_msgs.msg import LaserScan
from collections import deque
import math

from sensors.spin_node_helper import spin_nodes


# constants
POINTS_PER_PACKET = 12
PACKETS_PER_ROTATION = 56


class LidarForwarder(Node):
    def __init__(self):
        super().__init__('lidar')
        self.lidar_publisher = self.create_publisher(LidarRotation, 'sensors/lidar', 10)
        
        # Configure QoS for LiDAR data - use BEST_EFFORT reliability for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep most recent message
        )
        # Some helpful links for the lidar:
        # https://wiki.youyeetoo.com/en/Lidar/LD20
        # https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
        self.ld20_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            sensor_qos
        )
        self.full_rotation_of_points = deque(maxlen=PACKETS_PER_ROTATION * POINTS_PER_PACKET)
        self.count = 0
    
    def lidar_callback(self, msg):
        """
        Callback function for the lidar subscriber.
        Waits until a full rotation of points is received, then publishes them.
        """
        # self.get_logger().info(str(msg))
        # self.get_logger().info(f"got {len(msg.ranges)} points")
        # scan in new points
        points = self.process_laser_scan(msg)
        # self.full_rotation_of_points.extend(points)
        if self.count % 2 == 0:
            now = self.get_clock().now()
            # self.get_logger().info(f"sending points in {(now.nanoseconds - msg.header.stamp.nanosec)/1_000_000} ms")
            data_to_send = LidarRotation()
            data_to_send.points = points
            data_to_send.timestamp = msg.header.stamp.nanosec//1_000
            self.lidar_publisher.publish(data_to_send)
        self.count += 1
        
    def process_laser_scan(self, msg):
        """
        Process a LaserScan message and create lidar points.
        Finds the angle step and angle for each point based on the wiki formula.
        """
        points = []
        start_angle = msg.angle_min
        end_angle = msg.angle_max
        num_points = len(msg.ranges)
        if num_points == 0: return []
        # find angle step from wiki formula
        step = (end_angle - start_angle) / (num_points - 1) if num_points > 1 else 0
        for i, distance in enumerate(msg.ranges):
            # find angle based on wiki formula
            angle = start_angle + step * i
            weight = 255#msg.intensities[i] if i < len(msg.intensities) else 1.0
            if distance >= msg.range_min and distance <= msg.range_max:
                points.append(self.create_lidar_point(weight, distance, angle))
        return points
    
    def sanatize_number(self, number, maximum, default):
        """
        Sanatize a number to be within a range.
        
        Args:
            number: The number to sanatize
            maximum: The maximum value allowed for this field
            default: The default value if the number is invalid
        """
        try:
            if number is not None and not math.isnan(number) and not math.isinf(number):
                return min(maximum, max(0.0, number))
            else:
                return default
        except Exception:
            return default

    def create_lidar_point(self, weight, distance, angle):
        """
        Create a lidar point with the given weight, distance, and angle.
        """
        lidar_point = LiDAR()
        lidar_point.weight = int(self.sanatize_number(weight, 255, 255))
        lidar_point.distance = int(distance * 250)#self.sanatize_number(distance, 65535, 0))
        lidar_point.angle = float(angle) + math.pi/35 #self.sanatize_number(angle, 2 * math.pi, 0.0))
        return lidar_point


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(LidarForwarder())


if __name__ == '__main__':
    main()
