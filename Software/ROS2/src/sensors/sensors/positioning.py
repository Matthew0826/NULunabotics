import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from sensors.kalman import PositionTracker
# from sensors.other_kalman import find_robot_location
from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import Point, Rect, Acceleration, BeaconDistances

# TODO: put in the actual positions of the beacons
BEACON_POSITIONS = (
    (348, 0),  # Beacon 0
    (548, 0),    # Beacon 1
    (548, 200)   # Beacon 2
)

# TODO: change to actual initial position
# it would be (448, 100)
INITIAL_POSITION = (543, 5)
MAP_DIMENSIONS = (548, 487)

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('positioning')
        self.position_publisher = self.create_publisher(Point, '/sensors/position', 10)
        self.confidence_publisher = self.create_publisher(Rect, '/sensors/position_confidence', 10)
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
        
        # uses Kalman filter to track the position of the robot
        self.position_tracker = PositionTracker(BEACON_POSITIONS, INITIAL_POSITION, MAP_DIMENSIONS)
        self.previous_time = None
    
    def publish_position(self):
        # get the current time
        current_time = self.get_clock().now()
        # calculate the time since the last update
        if self.previous_time is None:
            dt = 0.1  # default to 0.1 seconds if this is the first update
        else:
            dt = (current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = current_time
        # update the position tracker with the new data
        self.position_tracker.update_tracker(dt, self.true_distances_to_beacons, self.linear_acceleration_vector)
        # publish the position and confidence
        position = Point()
        position.x = float(self.position_tracker.position[0])
        position.y = float(self.position_tracker.position[1])
        self.position_publisher.publish(position)
        confidence_rect = Rect()
        bounds = self.position_tracker.position_bounds
        confidence_rect.x1 = float(bounds[0])
        confidence_rect.y1 = float(bounds[1])
        confidence_rect.x2 = float(bounds[2])
        confidence_rect.y2 = float(bounds[3])
        self.confidence_publisher.publish(confidence_rect)
    
    def accel_callback(self, msg: Acceleration):
        self.angle_publisher.publish(Float32(data=msg.orientation))
        self.publish_position()
    
    def beacon_callback(self, msg: BeaconDistances):
        # convert the distances to true distances
        self.true_distances_to_beacons = (
            self.calculate_true_distance(msg.distance_0),
            self.calculate_true_distance(msg.distance_1),
            self.calculate_true_distance(msg.distance_2)
        )
        print(self.true_distances_to_beacons)
        self.publish_position()
        
    def calculate_true_distance(self, measured):
        # gotten from testing and through spreadsheet analysis
        # https://docs.google.com/spreadsheets/d/1d8JpcACcQQDd8Mt38nvdwsscAl1u1AYtAX2Nlg1wPS0/edit?usp=sharing
        # (go to Sheet 2)
        return (measured + 24) / 1.23


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(PositionPublisher())


if __name__ == '__main__':
    main()
