import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32
from collections import deque
import struct

from sensors.spin_node_helper import spin_nodes


# constants
BAUD_RATE = 9600
ESP_BOARD_ID = 4


class PowerData(Node):
    def __init__(self, port: str):
        super().__init__('power_data')
        self.voltage_publisher = self.create_publisher(Float32, 'sensors/voltage', 10)
        self.amps_publisher = self.create_publisher(Float32, 'sensors/amps', 10)
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(port)
        buffer = deque(maxlen=12)
        while True:
            data = ser.read()
            if len(data) < 1: continue
            buffer.append(data[0])
            # wait for header
            if bytes(buffer)[0:4] != b'\xFF\xFF\xFF\xFF': continue
            # wait for big enoug buffer
            if len(buffer) < 12: continue
            # find bytes for volts and amps floats
            volts_bytes = buffer[4:8]
            amps_bytes = buffer[8:12]
            voltage = struct.unpack('<f', volts_bytes)[0]  # Little-endian float
            current = struct.unpack('<f', amps_bytes)[0]
            print(f"Voltage: {voltage} V, Current: {current} A")
            # publish values for ROS
            self.voltage_publisher.publish(voltage)
            self.amps_publisher.publish(current)
            

def main(args=None):
    rclpy.init(args=args)
    spin_nodes(PowerData("/dev/ttyUSB0"))


if __name__ == '__main__':
    main()
