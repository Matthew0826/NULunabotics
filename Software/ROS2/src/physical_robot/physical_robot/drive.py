import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import serial
import datetime


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'website/controller',
            self.listener_callback,
            10
        )
        self.get_logger().info("Waiting for website's controller.")
        self.previous_data = [0, 0]
        
        self.serial_out = serial.Serial('/dev/ttyUSB2', 9600, timeout=1)
    

    def listener_callback(self, msg):
        decoded = json.loads(str(msg.data))
        delta = [decoded['x1'], decoded['y1']]
        if ((self.previous_data[0] - delta[0])**2 + (self.previous_data[1] - delta[1])**2) < 0.005625:
            self.previous_data = delta
            return
        self.previous_data = delta
        delta_bytes = [min(int((x + 1) * 256), 255).to_bytes(1, 'big') for x in delta]
        time_amount = datetime.datetime.now() - datetime.datetime.fromtimestamp(decoded['timestamp']/1000)
        self.get_logger().info("I heard: " + str(delta) + ". It took " + str(time_amount.microseconds/1000) + "ms.")
        self.serial_out.write(delta_bytes)
        self.serial_out.write(b"\r\n")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
