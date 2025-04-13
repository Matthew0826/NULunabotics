import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from lunabotics_interfaces.msg import Point, Motors

# the self driver agent action
from lunabotics_interfaces.action import SelfDriver

from navigation.pathfinder_helper import distance

from std_msgs.msg import Float32

import math
import time

# A subscriber to events which update the rover.
class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # create the action server
        self._action_server = ActionServer(
                self,
                SelfDriver,
                'self_driver',
                self.on_goal_updated
                )

        # initalize position
        self.position = Point()
        self.position.x = 448
        self.position.y = 100

        # initalize orientation
        self.orientation = 0.0

        # subscribe to position
        self.position_sub = subscribe(
            self,
            Point,
            'sensors/position'
            'on_position'
            )

        # subscribe to orientation
        self.orientation_sub = subscribe(
            self,
            Float32,
            'sensors/orientation'
            'on_orientation'
            )

        self.motor_pub = self.create_publisher(
            Motors,
            'website/controller',
            10
            )

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

    # when position updates
    def on_position(self, position):
        self.position = position

    # when orientation updates
    def on_orientation(self, orientation):
        self.set_orientation(orientation)

    # enforce orientation between 0 and 360
    def set_orientation(self, orientation: float):
        if (orientation < 0 or orientation > 360): 
            print(f"Invalid Orientation: {orientation}")
            return
        self.orientation = orientation

    # set the power of left and right front motors
    def set_motor_power(self, left_power, right_power):
        # message class
        msg = Motors()
        # one forth speed is plenty for now!
        msg.front_left_wheel = left_power / 4.0
        msg.front_right_wheel = right_power / 4.0
        msg.back_left_wheel = left_power / 4.0
        msg.back_right_wheel = right_power / 4.0
        msg.conveyor = 0.0
        msg.outtake = 0.0

        # Then publish it like usual
        self.motor_pub.publish(msg)
        # can save current motor power for future reference

    # orient the robot (global orientation)
    def to_orient(self, final_orientation: float):
        tolerance_degrees = 1
        while ((abs(self.orientation - final_orientation) + 360) % 360 > tolerance_degrees):
            # gap between where we are and want to be
            delta_orientation = self.orientation - final_orientation
            
            # delta_orientation/90 means full power when gap > 90
            # calculate motor power
            left_power = clamp(delta_orientation/90.0, -1.0, 1.0)
            right_power = -left_power
            # set motor power
            self.set_motor_power(left_power, right_power)

    # rotate the robot
    def to_rotate(self, delta: float):
        # new orientation (based on how much we rotate)
        final_orientation = self.orientation + delta

        # now go to new orientation
        self.to_orient(final_orientation)

    # drive forward in a straight line
    def to_line(self, length: float):
        
        # get the normalized vector of where we face
        new_x = self.position.x + (math.cos(math.radians(self.orientation)) * length)
        new_y = self.position.y + (math.sin(math.radians(self.orientation)) * length)

        # calculate the x, y we end up at if driving straight
        self.to_position(new_x, new_y)

    # move the robot from current to new position
    def to_position(self, x: float, y: float):
        tolerance_cm = 5.0
        # calculate orientation to face position
        self.face_position(x, y)
        # drive in a line until we reach the position
        while (distance(self.position, Point(x=x, y=y)) > tolerance_cm):
            self.set_motor_power(1, 1)
            time.sleep(0.05)

    # orient the rover to face a position
    def face_position(self, x: float, y: float):
        delta_x = x - self.position.x
        delta_y = y - self.position.y
        new_orientation = math.degrees(math.atan2(delta_y, delta_x))

        # rotate to new orientation
        self.to_orient(new_orientation)

    # drive along a path (list of points)
    def to_path(self, points):
        for point in points:
            self.to_position(point.x, point.y)

    # STRETCH: give a set of lines (graph) to drive along)
    # I think this would be a set of points actually
    def to_graph(self):
        pass

# self = the class to add subscription to
# getType = the type to send from the topic to subscriber
# topicName = the name of the event to subscribe to
# callbackName = the name of callback function on self to use
def subscribe(self, get_type, topic_name: str, callback_name: str):
    return self.create_subscription(
        get_type, 
        # the topic to subscribe t
        topic_name, 
        # the function called on an event
        self[callback_name],
        10,
        )

# clamp a value between a range
# should replace with sigmoid function later
def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)

    odometry = Odometry()

    # spin initializes the class as a node
    rclpy.spin(odometry)

    ### testing library ###
    # rotate bot to orientation of 90 degrees
    odometry.to_orient(90)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
