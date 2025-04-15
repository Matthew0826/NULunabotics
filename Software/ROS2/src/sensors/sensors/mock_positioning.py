import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from sensors.triangulation import find_robot_location
from sensors.serial_port_client import find_port

from lunabotics_interfaces.msg import Point
from lunabotics_interfaces.msg import Motors

import serial
import os

ESP_BOARD_ID = 2
BAUD_RATE = 9600

class SpacialDataPublisher(Node):

    def __init__(self):
        super().__init__('positioning')
        
        # create position publisher
        self.position_pub = self.create_publisher(Point, '/sensors/position', 10)
        
        # create orientation publisher
        self.orientation_pub = self.create_publisher(Float32, '/sensors/orientation', 10)
 
        self.motor_sub = self.create_subscriber(
                self,
                Motors,
                "physical_robot/motors",
                self.on_motor,
                )
        # keep track of positions
        self.x = 0
        self.y = 0
        self.orientation = 0

        # avoid unused variable warning (useless)
        self.motor_sub
                   
    def on_motor(self, motors):
        msg = Point()
        msg.x = self.x + 1
        msg.y = self.y + 1
        # publish mock position based on motor power
        self.position_pub.publish(msg)

        msg = (self.orientation + 1) % 360
        # publish mock orientation based on motor power
        self.orientation_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # create publisher node, start it up
    spacial_data_publisher = SpacialDataPublisher()
    rclpy.spin(spacial_data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spacial_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
