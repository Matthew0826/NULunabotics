import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float32

from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import Point
from lunabotics_interfaces.msg import Motors

import random

ESP_BOARD_ID = 2
BAUD_RATE = 9600

class SpacialDataPublisher(Node):

    def __init__(self):
        super().__init__('positioning')
        
        # create position publisher
        self.position_pub = self.create_publisher(Point, '/sensors/position', 10)
        
        # create orientation publisher
        self.orientation_pub = self.create_publisher(Float32, '/sensors/orientation', 10)
 
        self.motor_sub = self.create_subscription(
                Motors,
                "physical_robot/motors",
                self.on_motor,
                10)
        # keep track of positions
        self.x = 448.0
        self.y = 100.0
        self.orientation = float(random.randint(0, 360))
        
        self.motor_power_left = 0.0
        self.motor_power_right = 0.0
        
        self.timer = self.create_timer(0.1, self.timer_callback)
                   
    def on_motor(self, motors: Motors):
        self.motor_power_left = motors.front_left_wheel
        self.motor_power_right = motors.front_right_wheel
        # self.get_logger().info(f"got motors: {motors.front_left_wheel}, {motors.front_right_wheel}")
    
    def update_simulation(self):
        # Calculate average motor power for forward/backward movement
        average_power = (self.motor_power_left + self.motor_power_right) / 2.0
        
        # Calculate turning based on difference between motors
        motor_power_delta = self.motor_power_right - self.motor_power_left
        
        # Update orientation based on motor difference
        self.orientation += motor_power_delta * 1.0
        self.orientation = float(self.orientation % 360)
        
        # Move forward/backward based on average power with the corrected orientation
        self.x += math.cos(math.radians(360 - self.orientation)) * average_power * 3.0
        self.y += math.sin(math.radians(360 - self.orientation)) * average_power * 3.0
        
    def timer_callback(self):
        self.update_simulation()
        msg = Float32()
        msg.data = float(self.orientation)
        # publish mock orientation based on motor power
        self.orientation_pub.publish(msg)
        
        msg = Point()
        msg.x = float(self.x)
        msg.y = float(self.y)
        # publish mock position based on motor power
        self.position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(SpacialDataPublisher())


if __name__ == '__main__':
    main()
