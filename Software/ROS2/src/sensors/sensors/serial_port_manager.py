import serial
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.serialization import serialize_message, deserialize_message

from sensors.serial_port_observer import SerialPortObserver
from sensors.spin_node_helper import spin_nodes

# the universal BAUDRATE we are forcing for every board
# bit rate is AT LEAST baud rate
BAUD_RATE = 9600

# event that tells us when we have a new usb connected
# hires and fires serial port nodes 
# (responsible for connecting publishers and subscribers to serial ports)
class SerialPortManager(Node):
    def __init__(self):
        super().__init__('serial_port_manager')
        # the observer observers the serial ports and calls the callback functions when a port is added or removed
        self.observer = SerialPortObserver(self.add_serial_port, self.remove_serial_port)
        # publisher id to publisher
        self.ros_pubs: dict[int, Publisher] = {}
        # subscriber id to subscriber
        self.ros_subs: dict[int, Subscription] = {}
        # keep track of the serial ports
        self.serial_ports: dict[str, SerialPort] = {}
        self.serial_timer = self.create_timer(0.001, self.serial_timer_callback)
    
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
        self.port = port
        self.node = node
        self.byte_buffer = deque(maxlen=1024)
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
        self.byte_buffer.append(byte)
        # check for header of 0xAB 0xCD
        if len(self.byte_buffer) < 4: return
        if not (self.byte_buffer[0] == b'\xAB' and self.byte_buffer[1] == b'\xCD'):
            # remove the first byte
            self.byte_buffer.popleft()
            return

        # this is the number representing the type of data that will be published by ROS
        # (for example, id of 5 could mean the Motors message)
        pub_id_byte = self.byte_buffer[2]
        pub_id = int.from_bytes([pub_id_byte], byteorder='big')
        # record the publisher id
        if pub_id not in self.publisher_ids:
            self.publisher_ids.append(pub_id)
            print(f"Added message type id {pub_id}")
            
        # get message length
        message_length = int.from_bytes(self.byte_buffer[3:5], byteorder='big')
        # check if we have enough bytes to read the message
        if len(self.byte_buffer) < message_length + 5:  # 2 for the header, 1 for the pub_id, and 2 for the msg length
            # not enough bytes, wait for more
            return
        
        bytes_read = self.decode_message(pub_id, bytes(self.byte_buffer[3:]))
        if bytes_read != message_length:
            print(f"Warning: read {bytes_read} bytes, message claimed to contain {message_length} bytes (id: {pub_id})")
        # remove the bytes we have read
        for _ in range(bytes_read + 5):
            self.byte_buffer.popleft()

    def decode_message(self, pub_id: int, message: bytes):
        # get ros msg type
        ros_pub: Publisher = self.node.ros_pubs[pub_id]
        ros_msg_type = ros_pub.msg_type
        # deserialize
        ros_msg = deserialize_message(message, ros_msg_type)
        # publish to ros
        ros_pub.publish(ros_msg)
        # find size of the message
        serialized_msg: bytes = serialize_message(ros_msg)
        return len(serialized_msg)
    
    def write_msg_to_serial(self, pub_id: int, msg: any):
        # serialize the msg
        serialized_msg: bytes = serialize_message(msg)
        # write to serial
        self.serial.write(b'\xAB\xCD')
        self.serial.write(pub_id.to_bytes(1, byteorder='big'))
        self.serial.write(serialized_msg)


def main(args=None):
    rclpy.init(args=args)
    spin_nodes(SerialPortManager(), shutdown_callback=lambda n: n.close_serial_ports())


if __name__ == '__main__':
    main()
