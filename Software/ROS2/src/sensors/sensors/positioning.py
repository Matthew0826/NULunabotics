import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from sensors.triangulation import find_robot_location
from sensors.serial_port_client import find_port

from lunabotics_interfaces.msg import Point

import serial
import os
import math

ESP_BOARD_ID = 2
BAUD_RATE = 9600

import struct


class SpacialDataPublisher(Node):

    def __init__(self, port: str):
        super().__init__('positioning')
        self.position_publisher = self.create_publisher(Point, '/sensors/position', 10)
        self.angle_publisher = self.create_publisher(Float32, '/sensors/orientation', 10)
        self.get_logger().info("Spacial data publisher started")
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        # wait until serial is open
        self.get_logger().info(f"Serial port {port} opened successfully")
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
        buffer = []
        while True:
            new_byte = ser.read(1)
            if new_byte != b'':
                buffer.append(new_byte)
            if len(buffer) < 2: continue
            # print(buffer)
            if buffer[0] == b'\xFF':
                if buffer[1] == b'\xFF' and len(buffer) > 4:
                    print("got dist")
                    # read the anchor index and distance
                    anchor_index = int.from_bytes(buffer[2])
                    distance = int.from_bytes(buffer[4]) * 256 + int.from_bytes(buffer[3])
                    self.get_logger().info(f"Anchor index: {anchor_index}, Distance: {distance}")
                    # print("Anchor index: ", anchor_index, "Distance: ", distance)
                    if anchor_index <= 2 and distance < 10_000:
                        # publish new position to the topic
                        distances[int(anchor_index)] = distance
                        # if all distances are received, calculate the position
                        if -1 not in distances:
                            msg = Point()
                            position = find_robot_location(distances[0], distances[1], distances[2])
                            msg.x = position[0]
                            msg.y = position[1]
                            self.get_logger().info(str(distances))
                            distances = [-1, -1, -1]
                            self.position_publisher.publish(msg)
                elif buffer[1] == b'\xFD' and len(buffer) > 5:
                    print("got angle")
                    # reads the accelerometer value by
                    # decoding a 32 bit float from the buffer
                    angle = struct.unpack("<f", b''.join(buffer[2:6]))[0]
                    # self.get_logger().info(f"Angle: {angle}")
                    msg = Float32()
                    msg.data = float(angle)
                    self.angle_publisher.publish(msg)
            buffer = buffer[-6:] # keep only the last 6 bytes
            # print(buffer)


def main(args=None):
    rclpy.init(args=args)
    # port = find_port(ESP_BOARD_ID, os.getpid(), BAUD_RATE)
    spacial_data_publisher = SpacialDataPublisher("/dev/ttyUSB0")
    rclpy.spin(spacial_data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spacial_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
