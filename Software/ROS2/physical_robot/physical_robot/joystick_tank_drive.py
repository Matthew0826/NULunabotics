import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import serial
import serial.tools.list_ports


class JoystickTankDriveSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_tank_drive')
        self.subscription = self.create_subscription(
            String,
            'website/controller',
            self.listener_callback,
            10
        )
        self.get_logger().info("Waiting for website's controller.")
        
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))
        port = list(filter(lambda y: 'USB' in y, list(map(lambda x: x[0], sorted(ports)))))[0]
        print(port)
        self.serial_out = serial.Serial(port, 9600, timeout=1)
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
        try:
            decoded = json.loads(str(msg.data))
        except Exception as e:
            print(e)
            return
        left = decoded['y1']
        right = decoded['y2']
        if abs(left) < 0.05:
            left = 0
        if abs(right) < 0.05:
            right = 0
        is_delta_left_small = abs(self.prev_left - left) < 0.05
        is_delta_right_small = abs(self.prev_right - right) < 0.05
        if not is_delta_left_small:
            self.send_serial_packet(0, -left)
            self.send_serial_packet(1, -left)
            print(f'Writing to left: {int(((-(left/3.0)+1.0) * 127.5))}')
        if not is_delta_right_small:
            self.send_serial_packet(2, right)
            self.send_serial_packet(3, right)
            print(f'Writing to right: {int((((right/3.0)+1.0) * 127.5))}')
        self.prev_left = left
        self.prev_right = right


def main(args=None):
    rclpy.init(args=args)

    joystick_tank_drive_subscriber = JoystickTankDriveSubscriber()

    rclpy.spin(joystick_tank_drive_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_tank_drive_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
