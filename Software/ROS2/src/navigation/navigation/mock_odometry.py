

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float32

from lunabotics_interfaces.msg import Point, Motors
from lunabotics_interfaces.action import SelfDriver

import random
import math
import time

MOCK_SPEED = 5  # cm/s

# This mock odometry node pretends to be real odometry, but just says that the robot moved instead of actually moving it.
# It is used to test the planner and other nodes that depend on odometry.
class MockOdometry(Node):

    def __init__(self):
        super().__init__('mock_odometry')

        # create the action server
        self._action_server = ActionServer(
                self,
                SelfDriver,
                'self_driver',
                self.on_goal_updated
                )

        # initialize position
        self.position = Point()
        self.position.x = 448
        self.position.y = 100

        # initialize orientation randomly (they do that in the competition, although the robot won't really be able to tell immediately)
        self.orientation = float(random.randint(0, 360))

        self.position_publisher = self.create_publisher(Point, '/sensors/position', 10)
        self.angle_publisher = self.create_publisher(Float32, '/sensors/orientation', 10)
        
        # publish the initial position and orientation
        self.position_publisher.publish(self.position)
        self.angle_publisher.publish(self.orientation)

    # when someone calls this action
    # SelfDriver.action:
    # # Request
    # Point[] targets
    # ---
    # # Result
    # int64 time_elapsed_millis
    # ---
    # # Feedback
    # # from 0 to 1
    # float progress
    def on_goal_updated(self, goal_handle):
        feedback_msg = SelfDriver.Feedback()
        feedback_msg.progress = 0.0
        start_time = self.get_clock().now()

        # get points from the goal
        points = goal_handle.request.targets
        self.get_logger().info('Received goal with {0} points'.format(len(points)))
        self.to_path(points, goal_handle, feedback_msg)
        
        # send the result
        goal_handle.succeed()
        result = SelfDriver.Result()
        end_time = self.get_clock().now()
        result.time_elapsed_millis = (end_time - start_time).nanoseconds / 1_000_000
        return result
    
    # orient the robot (global orientation)
    def to_orient(self, final_orientation: float):
        self.orientation = (final_orientation + 360) % 360
        self.angle_publisher.publish(self.orientation)

    # rotate the robot
    def to_rotate(self, delta: float):
        # new orientation (based on how much we rotate)
        final_orientation = self.orientation + delta

        # now go to new orientation
        self.to_orient(final_orientation)

    # move the robot from current to new position
    def to_position(self, x: float, y: float, goal_handle, feedback_msg):
        self.face_position(x, y)
        distance = math.sqrt((self.position.x - x) ** 2 + (self.position.y - y) ** 2)
        for i in range(int(distance / MOCK_SPEED)):
            # move the robot
            self.position.x += MOCK_SPEED * math.cos(self.orientation)
            self.position.y += MOCK_SPEED * math.sin(self.orientation)
            # send out updated position
            self.position_publisher.publish(self.position)
            goal_handle.publish_feedback(feedback_msg)
            # pretend to take time to move
            time.sleep(1/MOCK_SPEED)
    
    # orient the rover to face a position
    def face_position(self, x: float, y: float):
        delta_x = self.x - x
        delta_y = self.y - y
        new_orientation = math.atan2(delta_y, delta_x)
        self.to_orient(new_orientation)

    # Drive along a path (list of points)
    def to_path(self, points, goal_handle, feedback_msg):
        for point in points:
            self.to_position(point, goal_handle, feedback_msg)
            feedback_msg.progress += 1 / len(points)


def main(args=None):
    rclpy.init(args=args)

    mock_odometry = MockOdometry()

    # spin initializes the class as a node
    rclpy.spin(mock_odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mock_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
