import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32

from sensors.triangulation import find_robot_location
from sensors.serial_port_client import find_port

from lunabotics_interfaces.msg import Point

import serial
import os

ESP_BOARD_ID = 2
BAUD_RATE = 9600

class SpacialDataPublisher(Node):

    def __init__(self, port: str):
        super().__init__('positioning')
        self.position_publisher = self.create_publisher(Point, '/sensors/position', 10)
        self.angle_publisher = self.create_publisher(Float32, '/sensors/orientation', 10)
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        # from esp c++ code:
        # // Protocol for communication with Raspberry Pi:
        # // Header: 0xFF
        # // If next byte is 0xFF, that means its sending a distance to a beacon
        # // Anchor index: 1 byte (0, 1, or 2)
        # // Distance: 2 bytes (in centimeters)
        # // 
        # // If the next bytes after the header was 0xFD, its sending an accelerometer value
        # // Right now it sends:
        # // Angle on Z axis: 4 bytes (float from 0.0 to 360.0)
        distances = [-1, -1, -1]
        while True:
            # read the header bytes
            ser.read_until(b'\xFF')
            header = ser.read(1)
            if header == b'\xFF':
                # read the anchor index and distance
                anchor_index = ser.read(1)
                distance = ser.read(2)
                distance = int.from_bytes(distance, byteorder='big')
                self.get_logger().info(f"Anchor index: {anchor_index}, Distance: {distance}")
                # publish new position to the topic
                distances[int(anchor_index)] = distance
                # if all distances are received, calculate the position
                if -1 not in distances:
                    msg = Point()
                    position = find_robot_location(distances[0], distances[1], distances[2])
                    msg.x = position[0]
                    msg.y = position[1]
                    # distances = [-1, -1, -1]
                    self.position_publisher.publish(msg)
            elif header == b'\xFD':
                # read the accelerometer value
                angle = ser.read(4)
                angle = float.from_bytes(angle, byteorder='big')
                self.get_logger().info(f"Angle: {angle}")
                msg = Float32()
                msg.data = angle
                self.angle_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    spacial_data_publisher = SpacialDataPublisher(find_port(ESP_BOARD_ID, os.getpid(), BAUD_RATE))
    rclpy.spin(spacial_data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spacial_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
