import rclpy
from rclpy.node import Node

from lunabotics_interfaces.msg import Motors
from sensors.spin_node_helper import spin_nodes

import serial

ESP_BOARD_ID = 1
BAUD_RATE = 9600

# ——— Channel assignments ———
# left front
LF_CH = 0
# left back
LB_CH = 1
# right front
RF_CH = 2
# right back
RB_CH = 3

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
            self.send_serial_packet(LF_CH, -left)
            self.send_serial_packet(LB_CH, -left)
        if not is_delta_right_small:
            self.send_serial_packet(RF_CH, right)
            self.send_serial_packet(RB_CH, right)
        self.prev_left = left
        self.prev_right = right


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(MotorDriver("/dev/ttyUSB0"))


if __name__ == '__main__':
    main()
