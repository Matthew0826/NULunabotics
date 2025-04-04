import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt16MultiArray
from collections import deque
import serial.tools.list_ports



# constants
VALUES_PER_POINT = 3  # distance and angle
POINTS_PER_PACKET = 12
PACKETS_PER_ROTATION = 56
BAUD_RATE = 19200


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('lidar')
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'sensors/lidar', 10)
        
        ports = serial.tools.list_ports.comports()
        # for port, desc, hwid in sorted(ports):
        #     print("{}: {} [{}]".format(port, desc, hwid))
        
        port = sorted(ports)[1][0]
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(port)
        deque_len = PACKETS_PER_ROTATION * POINTS_PER_PACKET * VALUES_PER_POINT + 4
        buffer = deque(maxlen=deque_len)
        while True:
            data = ser.read()
            if len(data) < 1: continue
            buffer.append(data[0])
            if bytes(buffer)[:4] != b'\xFF\xFF\xFF\xFF': continue
            if len(buffer) < deque_len: continue
            nums = [buffer[i*2]*256 + buffer[i*2 + 1] for i in range(4, deque_len//2)]
            
            data_to_send = UInt16MultiArray()
            data_to_send.data = nums
            self.publisher_.publish(data_to_send)
            print("I just sent " + str(len(nums)) + " values! Here's a sample: ")
            for i in range(5):
                print("weight: " + str(nums[i*3]))
            print("max weight:", max(nums[::3]))
            print("min weight:", min(nums[::3]))
            for i in range(5):
                print("angle: " + str(nums[i*3 + 1]/100))
            print("max angle:", max(nums[1::3])/100)
            print("min angle:", min(nums[1::3])/100)
            for i in range(5):
                print("distance: " + str(nums[i*3 + 2]))
            print("max distance:", max(nums[2::3]))
            print("min distance:", min(nums[2::3]))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
