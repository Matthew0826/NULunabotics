import rclpy
from rclpy.node import Node

# data type for lidar data
from std_msgs.msg import Float32MultiArray

# importing the JSON libraru
import json

# import math
import math

# A subscriber to events which update the rover.
class Odometry():

    def __init__(self):
        super().__init__('odometry')

        # initalize lidar
        self.lidar = 0

        # initalize position
        self.position = 0

        # initalize orientation
        self.orientation = 0

        # lidar: UInt16MultiArray, 
        # (weight, angle [360 = 36000], distance) 
        # position: Float32MultiArray,
        # (float x, float y)
        # 0, 0 is some corner idk
        # orientation: Float32,
        # rotation is on z axis
        # controller: JSON (String actually)

        # subscribe to lidar
        self.lidar_sub = subscribe(
            self,
            UInt16MultiArray,
            'sensor/lidar'
            'on_lidar'
            )

        # subscribe to position
        self.position_sub = subscribe(
            self,
            Float32MultiArray,
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
            String,
            'website/controller',
            10
            )
    
        # prevent unused variable warning (useless)
        self.lidar_sub
        self.position_sub
        self.orientation_sub
        self.motor_pub

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
    def set_orientation(self, int orientation):
        if (orientation < 0 or orientation > 360): 
            print("INVALID ORIENTATION: %d" %orientation)
            return

        # update since valid
        self.orientation = orientation

    # set the power of left and right front motors
    def set_motor_power(left_power, right_power):
        this.motor_pub.publish("{'y1': %d, 'y2': %d}" % left_power % right_power)
        # can save current motor power for future reference

    # orient the robot (global orientation)
    def to_orient(int final_orientation):
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
    def to_rotate(int delta):
        # new orientation (based on how much we rotate)
        final_orientation = this.orientation + delta

        # now go to new orientation
        this.to_orient(final_orientation)

    # drive forward in a straight line
    def to_line(int length):
        
        # get the normalized vector of where we face
        new_x = this.position.x + (math.cos(this.orientation) * length)
        new_y = this.position.y + (math.sin(this.orientation) * length)

        # calculate the x, y we end up at if driving straight
        to_position(new_x, new_y)

    # move the robot from current to new position
    def to_position(int x, int y):

        # tolerance
        tolerance = 1

        # calculate orientation to face position
        face_position(x, y)

        # drive in a line until we reach the position
        while (this.position - current_position) > 1):
            self.set_motor_power(1, 1)

    # orient the rover to face a position
    def face_position(int x, int y):
        pass
        # use atan2

    # STRETCH: give a set of lines (graph) to drive along)
    # I think this would be a set of points actually
    def to_graph():
        pass

# self = the class to add subscription to
# getType = the type to send from the topic to subscriber
# topicName = the name of the event to subscribe to
# callbackName = the name of callback function on self to use
def subscribe(self, getType, str topicName, str callbackName):
    return self.create_subscription(
        getType, 
        # the topic to subscribe t
        topicName, 
        # the function called on an event
        self[callbackName]
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
