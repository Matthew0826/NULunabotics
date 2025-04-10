import rclpy
from rclpy.node import Node

# data type for lidar data & coordinates
from lunabotics_interfaces.msg import LidarRotation, Point, Motor

# a path data type
from lunabotics_interfaces.srv import Path

# the self driver agent action
from lunabotics_interfaces.actions import SelfDriver

# importing the JSON libraru
import json

# import math
import math

# A subscriber to events which update the rover.
class Odometry(Node):

    def __init__(self):
        super().__init__('odometry')

        # create the action server
        self._action_server = ActionServer(
                self,
                SelfDriver
                'self_driver',
                self.on_goal_updated
                )

        # initalize lidar
        self.lidar = 0

        # initalize position
        self.position = 0

        # initalize orientation
        self.orientation = 0

        # lidar: LidarRotation UInt16MultiArray, 
        # (weight, angle [360 = 36000], distance) 
        # position: Point Float32MultiArray,
        # (float x, float y)
        # 0, 0 is some corner idk
        # orientation: Float32,
        # rotation is on z axis
        # controller: JSON (String actually)

        # subscribe to lidar
        self.lidar_sub = subscribe(
            self,
            LidarRotation,
            'sensor/lidar'
            'on_lidar'
            )

        # subscribe to position
        self.position_sub = subscribe(
            self,
            Point,
            'sensor/position'
            'on_position'
            )

        # subscribe to orientation
        self.orientation_sub = subscribe(
            self,
            Float32,
            'sensor/orientation'
            'on_orientation'
            )

        # publish JSON String Control
        # Example (Left forward, Right backward): 
        # Thus it turns right in this case
        # set the motor power to these values
        # "{"y1": 1, "y2": -1}"

        self.motor_pub = self.create_publisher(
            Motors,
            'website/controller',
            10
            )
    
        # prevent unused variable warning (useless)
        self.lidar_sub
        self.position_sub
        self.orientation_sub
        self.motor_pub

    # when someone calls this action
    # takes in self and goal to be handled
    def on_goal_updated(self, goal):

        # get the feedback
        feedback_msg = SelfDriver.Feedback()

        # update current progress
        feedback_msg.progress += 1

        # do the thing
        while (False):
            
            # publish feedback
            goal.publish_feedback(feedback_msg)
            time.sleep(1)
    
        # we did it
        goal.succeed()

        # set the result
        result = SelfDriver.Result()
        result.result = 0
        return result


    # when lidar updates
    def on_lidar(self, lidar):
        self.lidar = lidar

    # when position updates
    def on_position(self, position):
        self.position = position

    # when orientation updates
    def on_orientation(self, orientation):
        self.set_orientation(orientation)

    # called everytime SOMETHING is updated
    def listener_callback(self, msg):
        msg.data = 'Hello World: %d' % msg
        
        # test increment orientation
        self.orientation += 1

    # enforce orientation between 0 and 360
    def set_orientation(self, orientation: float):
        if (orientation < 0 or orientation > 360): 
            print("INVALID ORIENTATION: %d" %orientation)
            return

        # update since valid
        self.orientation = orientation

    # set the power of left and right front motors
    def set_motor_power(left_power, right_power):
        # message class
        msg = Motors()
        msg.front_left_wheel = left_power
        msg.front_right_wheel = right_power
        msg.back_left_wheel = left_power
        msg.back_right_wheel = right_power
        msg.conveyor = 0.0
        msg.outtake = 0.0

        # Then publish it like usual
        self.motor_pub.publish(msg)
        # can save current motor power for future reference

    # orient the robot (global orientation)
    def to_orient(final_orientation: float):
        # tolerance
        tolerance = 1
        
        while (abs(self.orientation - final_orientation) > tolerance):
            # gap between where we are and want to be
            delta_orientation = self.orientation - final_orientation

            # delta_orientation/90 means full power when gap > 90

            # calculate right motor power
            left_power = clamp(delta_orientation/90, -1, 1)

            # calculate left motor power
            right_power = -left_power

            # set motor power
            self.set_motor_power(left_power, right_power)

            # done

    # rotate the robot
    def to_rotate(delta: float):
        # new orientation (based on how much we rotate)
        final_orientation = this.orientation + delta

        # now go to new orientation
        this.to_orient(final_orientation)

    # drive forward in a straight line
    def to_line(length: float):
        
        # get the normalized vector of where we face
        new_x = this.position.x + (math.cos(this.orientation) * length)
        new_y = this.position.y + (math.sin(this.orientation) * length)

        # calculate the x, y we end up at if driving straight
        to_position(new_x, new_y)

    # move the robot from current to new position
    def to_position(x: float, y: float):

        # tolerance
        tolerance = 1

        # calculate orientation to face position
        face_position(x, y)

        # drive in a line until we reach the position
        while ((this.position - current_position) > 1):
            self.set_motor_power(1, 1)

    # orient the rover to face a position
    def face_position(x: float, y: float):
        delta_x = self.x - x
        delta_y = self.y - y

        # use atan2
        new_orientation = math.atan2(y, x)

        # rotate to new orientation
        self.to_orient(new_orientation)

    # Drive along a path (List of Points)
    def to_path(points):
        for point in points:
            self.to_position(point)

    # STRETCH: give a set of lines (graph) to drive along)
    # I think this would be a set of points actually
    def to_graph():
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
