import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float32

from lunabotics_interfaces.msg import Point
from lunabotics_interfaces.action import SelfDriver

from sensors.spin_node_helper import spin_nodes

import random
import math
import time

MOCK_SPEED = 5  # cm/s
MOCK_ROTATION_SPEED = 5  # degrees/s


class MockOdometry(Node):
    """
    This mock odometry node pretends to be real odometry, but just says that the robot moved instead of actually moving it.
    It is used to test the planner and other nodes that depend on odometry.
    """

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
        self.position.x = 448.0
        self.position.y = 100.0

        # initialize orientation randomly (they do that in the competition, although the robot won't really be able to tell immediately)
        self.orientation = float(random.randint(0, 360))

        self.position_publisher = self.create_publisher(Point, '/sensors/position', 10)
        self.angle_publisher = self.create_publisher(Float32, '/sensors/orientation', 10)
        
        # publish the initial position and orientation
        self.position_publisher.publish(self.position)
        self.publish_angle()
    
    def publish_angle(self):
        """Prented to update the real angle of the robot."""
        angle_msg = Float32()
        angle_msg.data = self.orientation
        self.angle_publisher.publish(angle_msg)

    def on_goal_updated(self, goal_handle):
        """     
        This function is called when someone calls this action to move the robot.
        SelfDriver.action:
        # Request
        Point[] targets
        ---
        # Result
        int64 time_elapsed_millis
        ---
        # Feedback
        # from 0 to 1
        float progress
        """
        
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
        result.time_elapsed_millis = (end_time - start_time).nanoseconds // 1_000_000
        return result
    
    # orient the robot (global orientation)
    def to_orient(self, final_orientation: float):
        """Rotates the robot to the given orientation slowly to simulate real rotation. Final orientation is in degrees."""
        # normalize the angle to be between 0 and 360
        # to find degrees that needs to be rotated by
        delta = final_orientation - self.orientation
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        # loop through a series of discrete steps to simulate rotation
        steps = int(abs(delta) / MOCK_ROTATION_SPEED)
        for i in range(steps):
            # rotate the robot
            self.orientation += delta / steps
            # send out updated position
            self.publish_angle()
            # pretend to take time to rotate
            time.sleep(1/(MOCK_ROTATION_SPEED*2))
        # set the final orientation
        self.orientation = final_orientation
        self.publish_angle()

    # rotate the robot
    def to_rotate(self, delta: float):
        """Rotates the robot to the given orientation slowly to simulate real rotation. delta is in degrees."""
        # new orientation (based on how much we rotate)
        final_orientation = self.orientation + delta

        # now go to new orientation
        self.to_orient(final_orientation)

    # move the robot from current to new position
    def to_position(self, x: float, y: float, goal_handle, feedback_msg):
        """Moves the robot to the given position slowly to simulate real movement. x and y are in cm."""
        # to fix the issue on the website where the robot was beginning to rotate, but it was rendered in the wrong position
        self.position_publisher.publish(self.position)
        # orient the robot to face the new position
        self.face_position(x, y)
        distance = math.sqrt((self.position.x - x) ** 2 + (self.position.y - y) ** 2)
        # move robot in discrete steps to simulate movement
        for i in range(int(distance / MOCK_SPEED)):
            # move the robot
            self.position.x += MOCK_SPEED * math.cos(math.radians(self.orientation))
            self.position.y += MOCK_SPEED * math.sin(math.radians(self.orientation))
            # send out updated position
            self.position_publisher.publish(self.position)
            goal_handle.publish_feedback(feedback_msg)
            # pretend to take time to move
            time.sleep(1/(MOCK_SPEED*2))
    
    def face_position(self, x: float, y: float):
        """
        Orients the rover to face a position.
        Calculates the angle and rotates the robot to face the given x, y coordinate.
        x and y are in cm.
        """
        delta_x = x - self.position.x
        delta_y = y - self.position.y
        new_orientation = math.degrees(math.atan2(delta_y, delta_x))
        self.to_orient(new_orientation)

    def to_path(self, points, goal_handle, feedback_msg):
        """Drive along a path (list of points). Publishes feedback as it goes about its progress."""
        for i, point in enumerate(points):
            self.to_position(point.x, point.y, goal_handle, feedback_msg)
            feedback_msg.progress = (i + 1) / len(points)
            goal_handle.publish_feedback(feedback_msg)
        feedback_msg.progress = 1.0
        goal_handle.publish_feedback(feedback_msg)


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(MockOdometry())


if __name__ == '__main__':
    main()
