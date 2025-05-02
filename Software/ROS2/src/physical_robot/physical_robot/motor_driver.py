import rclpy
from rclpy.node import Node

from lunabotics_interfaces.msg import Motors

import serial
import os

ESP_BOARD_ID = 1
BAUD_RATE = 9600

class MotorDriver(Node):
    def __init__(self, port: str):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            Motors,
            'physical_robot/motors',
            self.listener_callback,
            10
        )
        self.get_logger().info("Waiting for website's controller.")
        
        self.serial_out = serial.Serial(port, BAUD_RATE, timeout=1)
        self.prev_left = 0
        self.prev_right = 0
    
    def send_serial_packet(self, motor: int, value: float):
        # send header
        try:
            self.serial_out.write(b'\xFF\xFF')
            # send data
            self.serial_out.write(motor.to_bytes(1))
            self.serial_out.write(int((((value/4.0)+1.0) * 127.5)).to_bytes(1))
        except Exception as e:
            pass

    def listener_callback(self, msg):
        left = msg.left_wheels
        right = msg.right_wheels
        if abs(left) < 0.05:
            left = 0
        if abs(right) < 0.05:
            right = 0
        is_delta_left_small = abs(self.prev_left - left) < 0.05
        is_delta_right_small = abs(self.prev_right - right) < 0.05
        if not is_delta_left_small:
            self.send_serial_packet(0, -left)
            self.send_serial_packet(1, -left)
        if not is_delta_right_small:
            self.send_serial_packet(2, right)
            self.send_serial_packet(3, right)
        self.prev_left = left
        self.prev_right = right


def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriver("/dev/ttyUSB0")

    rclpy.spin(motor_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
