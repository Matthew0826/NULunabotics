import rclpy
from rclpy.node import Node
import math

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

# tag offset on the robot
UWB_TAG_OFFSET = (23.0, 40.0, 6.5)
# the height of the shelf relative to the floor
SHELF_HEIGHT = 0.0
UWB_ANCHOR_HEIGHT = SHELF_HEIGHT - UWB_TAG_OFFSET[1]

def local_to_global_offset(local_offset, robot_yaw_degrees):
    # Convert yaw from degrees to radians
    yaw_rad = math.radians(robot_yaw_degrees)
    
    # Extract local coordinates
    local_x, local_y = local_offset
    
    # Apply 2D rotation matrix
    global_x = local_x * math.cos(yaw_rad) - local_y * math.sin(yaw_rad)
    global_y = local_x * math.sin(yaw_rad) + local_y * math.cos(yaw_rad)
    
    return (global_x, global_y)

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
        self.orientation = 0.0
    
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
        offset = local_to_global_offset((UWB_TAG_OFFSET[0], UWB_TAG_OFFSET[2]), self.orientation)
        # publish the position and confidence
        position = Point()
        position.x = float(self.position_tracker.position[0] + offset[0])
        position.y = float(self.position_tracker.position[1] + offset[1])
        self.position_publisher.publish(position)
        confidence_rect = Rect()
        bounds = self.position_tracker.position_bounds
        confidence_rect.x1 = float(bounds[0] + offset[0])
        confidence_rect.y1 = float(bounds[1] + offset[1])
        confidence_rect.x2 = float(bounds[2] + offset[0])
        confidence_rect.y2 = float(bounds[3] + offset[1])
        self.confidence_publisher.publish(confidence_rect)
    
    def accel_callback(self, msg: Acceleration):
        self.angle_publisher.publish(Float32(data=msg.orientation))
        self.orientation = msg.orientation
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
        # account for the height of the tag
        xz_plane_distance = math.sqrt(abs(measured**2 - UWB_ANCHOR_HEIGHT**2))
        
        # gotten from testing and through spreadsheet analysis
        # https://docs.google.com/spreadsheets/d/1d8JpcACcQQDd8Mt38nvdwsscAl1u1AYtAX2Nlg1wPS0/edit?usp=sharing
        # (go to Sheet 2)
        return (xz_plane_distance + 24) / 1.23


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(PositionPublisher())


if __name__ == '__main__':
    main()
