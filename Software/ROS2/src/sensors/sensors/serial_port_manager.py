import serial
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.serialization import serialize_message, deserialize_message

from sensors.serial_port_observer import SerialPortObserver
from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import Acceleration

# the universal BAUDRATE we are forcing for every board
# bit rate is AT LEAST baud rate
BAUD_RATE = 9600

LITTLE_ENDIAN = b'\x00\x01'
BIG_ENDIAN = b'\x00\x00'


def deserialize(message: bytes, ros_type: any):
    # the first 2 bytes of the serialized message are 0x0001 if the message is little-endian, and 0x0000 if the message is big-endian
    # the next 2 bytes are always 0
    
    # add 4 bytes to the beginning of the message to represent little-endianess
    message = LITTLE_ENDIAN + b'\x00\x00' + message
    # deserialize the message
    deserialized_msg = deserialize_message(message, ros_type)
    return deserialized_msg


def serialize(message: any):
    # serialize the message
    serialized_msg = serialize_message(message)
    # remove the first 4 bytes
    serialized_msg = serialized_msg[4:]
    return serialized_msg

def are_bytes_delimiter(bytes: bytes):
    return bytes == b'\xAB\xCD'


# event that tells us when we have a new usb connected
# hires and fires serial port nodes 
# (responsible for connecting publishers and subscribers to serial ports)
class SerialPortManager(Node):
    def __init__(self):
        super().__init__('serial_port_manager')
        # publisher id to publisher
        self.ros_pubs: dict[int, Publisher] = {}
        # subscriber id to subscriber
        self.ros_subs: dict[int, Subscription] = {}
        # keep track of the serial ports
        self.serial_ports: dict[str, SerialPort] = {}
        self.serial_timer = self.create_timer(0.001, self.serial_timer_callback)
        
        # the observer observers the serial ports and calls the callback functions when a port is added or removed
        self.observer = SerialPortObserver(self.add_serial_port, self.remove_serial_port)
    
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
    # this is called when the serial port is connected
    def add_serial_port(self, port: str):
        if port not in self.serial_ports:
            # dependency injection of this node into the serial port
            new_port = SerialPort(self, port)
            new_port.open()
            # add the serial port to the map
            self.serial_ports[port] = new_port
            print(f"Added serial port {port}")
        else:
            print(f"Serial port {port} already exists")
    
    # remove a serial port from the list
    # this is called when the serial port is disconnected
    def remove_serial_port(self, port: str):
        if port in self.serial_ports:
            # close the serial port
            self.serial_ports[port].serial.close()
            # remove the serial port from the list
            del self.serial_ports[port]
            print(f"Removed serial port {port}")
        else:
            print(f"Serial port {port} does not exist")
            
    def on_new_msg(self, pub_id: int, msg: any):
        # this is called when a new message is received from a ros subscription
        # find all serial ports with matching publisher id
        for port, serial_obj in self.serial_ports.items():
            if pub_id in serial_obj.publisher_ids:
                # write the message to the serial port
                serial_obj.write_msg_to_serial(pub_id, msg)
                print(f"Sent message {msg} to serial port {port} with id {pub_id}")
        else:
            print(f"Id {pub_id} not found in any serial ports")

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
            # create a new ros subscription
            # it calls the on_new_msg function with the pub_id
            new_sub = self.create_subscription(ros_msg_type, topic, lambda msg: self.on_new_msg(pub_id, msg), 10)
            self.ros_pubs[pub_id] = new_sub
            print(f"Added ROS subscriber {topic} with ID {pub_id}")
        else:
            print(f"Subscriber ID {pub_id} already exists")


class SerialPort:
    def __init__(self, node: Node, port):
        self.previous_byte = b''
        self.port = port
        self.node = node
        self.publisher_ids = []
        self.is_open = False
        self.serial = None

    def open(self):
        if not self.is_open:
            try:
                self.serial = serial.Serial(self.port, BAUD_RATE, timeout=2)
                self.is_open = True
                print(f"Opened serial port: {self.port}")
            except serial.SerialException as e:
                print(f"Failed to open serial port {self.port}: {e}")
    
    def parse_byte(self, byte: bytes):
        # check for delimiter to indicate start of message
        # print(f"Received byte: {byte}")
        if not are_bytes_delimiter(self.previous_byte + byte):
            self.previous_byte = byte
            return
        self.previous_byte = byte

        # read 3 more bytes of header
        header_bytes = self.serial.read(3)
        if len(header_bytes) < 3:
            print(f"Warning: not enough bytes read from serial port {self.port}")
            return
        pub_id_byte = [header_bytes[0]]
        message_length = int.from_bytes([header_bytes[2]]) << 8 | int.from_bytes([header_bytes[1]])
        print(f"Header bytes: {header_bytes}, pub_id: {pub_id_byte}, message_length: {message_length}")

        # this is the number representing the type of data that will be published by ROS
        # (for example, id of 5 could mean the Motors message)
        pub_id = int.from_bytes(pub_id_byte)
        print(f"Publisher ID: {pub_id}")
        # record the publisher id
        if pub_id not in self.publisher_ids:
            # check if the pub_id exists
            if pub_id not in self.node.ros_pubs:
                print(f"Publisher ID {pub_id} not found in ROS publishers")
                return
            self.publisher_ids.append(pub_id)
            print(f"Added message type id {pub_id}")
        
        # read in message_length bytes
        message_bytes = self.serial.read(message_length)
        print(f"Read {len(message_bytes)} bytes from serial port {self.port} with id {pub_id}")
        bytes_read = self.decode_message(pub_id, message_bytes)
        if bytes_read != message_length:
            print(f"Warning: read {bytes_read} bytes, message claimed to contain {message_length} bytes (id: {pub_id})")

    def decode_message(self, pub_id: int, message: bytes):
        if pub_id not in self.node.ros_pubs:
            print(f"Publisher ID {pub_id} not found in ROS publishers")
            return 0
        # get ros msg type
        ros_pub: Publisher = self.node.ros_pubs[pub_id]
        ros_msg_type = ros_pub.msg_type
        # deserialize
        ros_msg = deserialize(message, ros_msg_type)
        print(f"Deserialized message: {ros_msg}")
        # publish to ros
        ros_pub.publish(ros_msg)
        # find size of the message
        serialized_msg: bytes = serialize(ros_msg)
        return len(serialized_msg)
    
    def write_msg_to_serial(self, pub_id: int, msg: any):
        # serialize the msg
        serialized_msg: bytes = serialize(msg)
        
        # write to serial
        # write the header
        self.serial.write(b'\xAB\xCD')
        # write the pub_id
        self.serial.write(pub_id.to_bytes(1))
        # write the length of the message
        msg_length = len(serialized_msg)
        self.serial.write(msg_length.to_bytes(2))
        # write the message itself
        self.serial.write(serialized_msg)
        print(f"Sent message {msg} to serial port {self.port} with id {pub_id} and length {msg_length}")


def main(args=None):
    rclpy.init(args=args)
    manager = SerialPortManager()
    manager.add_ros_pub(0, Acceleration, '/test1')
    spin_nodes(manager, shutdown_callback=lambda n: n.close_serial_ports())


if __name__ == '__main__':
    main()
