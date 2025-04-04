import rclpy
from rclpy.node import Node
import serial
from lunabotics_interfaces.msg import LidarRotation
from lunabotics_interfaces.msg import LiDAR
from sensors.serial_port_client import find_port
from collections import deque
import math
import os



# constants
VALUES_PER_POINT = 3  # distance and angle
POINTS_PER_PACKET = 12
PACKETS_PER_ROTATION = 56
BAUD_RATE = 19200
ESP_BOARD_ID = 3


class LidarPublisher(Node):
    def __init__(self, port: str):
        super().__init__('lidar')
        self.publisher_ = self.create_publisher(LidarRotation, 'sensors/lidar', 10)
        
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
            points = [self.create_lidar_point(nums[i], nums[i + 1], nums[i + 2]) for i in range(0, len(nums), VALUES_PER_POINT)]
            data_to_send = LidarRotation()
            data_to_send.points = points
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
    
    def create_lidar_point(self, weight, distance, angle):
        lidar_point = LiDAR()
        lidar_point.weight = weight
        lidar_point.distance = distance
        lidar_point.angle = math.PI * (angle/100.0) / 180.0
        return lidar_point


def main(args=None):
    rclpy.init(args=args)

    lidar_publisher = LidarPublisher(find_port(ESP_BOARD_ID, os.getpid(), BAUD_RATE))

    rclpy.spin(lidar_publisher)

    lidar_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
