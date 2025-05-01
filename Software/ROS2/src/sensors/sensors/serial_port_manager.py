import serial
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
import rclpy.serialization
from rclpy.subscription import Subscription
from rclpy.serialization import serialize_message, deserialize_message

from lunabotics_interfaces.msg import Motors
from std_msgs.msg import Float32

from sensors.serial_port_observer import SerialPortObserver

# the universal BAUDRATE we are forcing for every board
# bit rate is AT LEAST baud rate
BAUD_RATE = 9600

# event that tells us when we have a new usb connected
# hires and fires serial port nodes 
# (responsible for connecting publishers and subscribers to serial ports)
class SerialPortManager(Node):
    def __init__(self):
        super().__init__('serial_port_manager')
        self.observer = SerialPortObserver(self.add_serial_port, self.remove_serial_port)
        self.serial_ports: dict[str, SerialPort] = {}
        # publisher id to publisher
        self.ros_pubs: dict[int, Publisher] = {}
        # subscriber id to subscriber
        self.ros_subs: dict[int, Subscription] = {}
        self.serial_timer = self.create_timer(0.05, self.serial_timer_callback)
    
    def serial_timer_callback(self):
        # check if any serial ports are open
        for port, serial_obj in self.serial_ports.items():
            if not serial_obj.is_open: continue
            # read from the serial port
            byte = serial_obj.serial.read(1)
            if byte != b'':
                serial_obj.parse_byte(byte)
    
    def close_serial_ports(self):
        for port, serial_obj in self.serial_ports.items():
            serial_obj.serial.close()
    
    # add a serial port to the list
    def add_serial_port(self, port: str):
        if port not in self.serial_ports:
            new_port = SerialPort(self, port)
            new_port.open()
            # add the serial port to the list
            self.serial_ports[port] = new_port
            print(f"Added serial port {port}")
        else:
            print(f"Serial port {port} already exists")
    
    def remove_serial_port(self, port: str):
        if port in self.serial_ports:
            # close the serial port
            self.serial_ports[port].serial.close()
            # remove the serial port from the list
            del self.serial_ports[port]
            print(f"Removed serial port {port}")
        else:
            print(f"Serial port {port} does not exist")

    # add a ros publisher to the list
    def add_ros_pub(self, pub_id: int, ros_msg_type: any, topic: str):
        if pub_id not in self.ros_pubs:
            new_pub = self.create_publisher(ros_msg_type, topic, 10)
            # add the publisher to the list
            self.ros_pubs[pub_id] = new_pub
            print(f"Added ROS publisher {topic} with ID {pub_id}")
        else:
            print(f"Publisher ID {pub_id} already exists")
            
    # add a ros subscriber to the list
    def add_ros_sub(self, pub_id: int, ros_msg_type: any, topic: str):
        if pub_id not in self.ros_subs:
            new_sub = self.create_subscription(ros_msg_type, topic, self., 10)
            self.ros_pubs[pub_id] = new_sub
            print(f"Added ROS subscriber {topic} with ID {pub_id}")
        else:
            print(f"Subscriber ID {pub_id} already exists")

class SerialPort:
    def __init__(self, node: Node, port):
        self.port = port
        self.is_open = False
        self.serial = None
        self.node = node
        self.byte_buffer = deque(maxlen=1024)

    def open(self):
        if not self.is_open:
            try:
                self.serial = serial.Serial(self.port, BAUD_RATE, timeout=2)
                self.is_open = True
                print(f"Opened serial port: {self.port}")
            except serial.SerialException as e:
                print(f"Failed to open serial port {self.port}: {e}")
    
    def parse_byte(self, byte: bytes):
        self.byte_buffer.append(byte)
        # check if we have a complete message
        # check for header of 0xAB 0xCD
        if len(self.byte_buffer) < 2: return
        if not (self.byte_buffer[0] == b'\xAB' and self.byte_buffer[1] == b'\xCD'):
            # remove the first byte
            self.byte_buffer.popleft()
            return
        # TODO: do we need to check for the end of the message?
        self.decode_message(bytes(self.byte_buffer[2:]))

    def decode_message(self, message: bytes):
        # first byte will be the board id
        if len(message) < 1:
            print("Message too short to decode")
            return None
        # this is the number representing the type of data that will be published by ROS
        # (for example, id of 2 could mean the Motors message)
        pub_id_byte = message[0]
        # convert to int
        pub_id = int.from_bytes([pub_id_byte], byteorder='big')
        # check if the board id is the same as the one we expect
        if pub_id not in self.publisher_ids:
            self.publisher_ids.append(pub_id)
            print(f"Added message type id {pub_id}")
    
        # get ros msg type
        ros_pub: Publisher = self.node.ros_pubs[pub_id]
        ros_msg_type = ros_pub.msg_type
        # deserialize
        ros_msg = deserialize_message(message[1:], ros_msg_type)
        # publish to ros
        ros_pub.publish(ros_msg)
    
    def write_msg_to_serial(self, msg: any):
        # serialize the msg
        serialized_msg: bytes = serialize_message(msg)
        # write to serial
        self.serial.write(serialized_msg)


def main(args=None):
    rclpy.init()
    serial_port_manager = SerialPortManager()
    
    try:
        rclpy.spin(serial_port_manager)
    except KeyboardInterrupt:
        serial_port_manager.get_logger().info('Keyboard interrupt detected, shutting down...')
    finally:
        serial_port_manager.destroy_node()
        rclpy.shutdown()
        serial_port_manager.close_serial_ports()


if __name__ == '__main__':
    main()
