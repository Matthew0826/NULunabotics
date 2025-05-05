import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from sensors.kalman_option_2 import find_robot_location
# from sensors.other_kalman import find_robot_location
from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import Point, Acceleration, BeaconDistances

ESP_BOARD_ID = 2
BAUD_RATE = 9600

class SpacialDataPublisher(Node):

    def __init__(self):
        super().__init__('positioning')
        self.position_publisher = self.create_publisher(Point, '/sensors/position', 10)
        self.angle_publisher = self.create_publisher(Float32, '/sensors/orientation', 10)
        self.acceleration_subscription = self.create_subscription(
            Acceleration,
            '/sensors/acceleration',
            self.accel_callback,
            10)
        self.beacon_subscription = self.create_subscription(
            BeaconDistances,
            '/sensors/beacon_distances',
            self.beacon_callback,
            10)
        
        # ignore z axis on acceleration
        self.linear_acceleration_vector = (0, 0)
        # distances in cm to each beacon
        self.true_distances_to_beacons = (0, 0, 0)
    
    def publish_position(self):
        position = Point()
        found_location = find_robot_location(*self.true_distances_to_beacons, *self.linear_acceleration_vector)
        position.x = found_location[0]
        position.y = found_location[1]
        self.position_publisher.publish(position)
    
    def accel_callback(self, msg: Acceleration):
        self.angle_publisher.publish(msg.orientation)
        self.linear_acceleration_vector = (msg.acceleration_x, msg.acceleration_y)
        self.publish_position()
    
    def beacon_callback(self, msg: BeaconDistances):
        # convert the distances to true distances
        self.true_distances_to_beacons = (
            self.calculate_true_distance(msg.distance_0),
            self.calculate_true_distance(msg.distance_1),
            self.calculate_true_distance(msg.distance_2)
        )
        self.publish_position()
        
    def calculate_true_distance(self, measured):
        # gotten from testing and through spreadsheet analysis
        # https://docs.google.com/spreadsheets/d/1d8JpcACcQQDd8Mt38nvdwsscAl1u1AYtAX2Nlg1wPS0/edit?usp=sharing
        # (go to Sheet 2)
        return (measured + 24) / 1.23


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(SpacialDataPublisher())


if __name__ == '__main__':
    main()
