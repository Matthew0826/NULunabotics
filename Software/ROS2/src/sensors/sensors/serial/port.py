from rclpy.node import Node
from rclpy.publisher import Publisher
import threading
import serial
from .serializer import deserialize, serialize
from .arduino_uploader import upload_sketch

# the universal BAUDRATE we are forcing for every board
# note: bit rate is AT LEAST baud rate
BAUD_RATE = 19200
DELIMITER = b'\xAB\xCD'

class SerialPort:
    def __init__(self, node: Node, port):
        self.port = port
        self.node = node
        self.publisher_ids = []
        self.is_open = False
        self.serial = None

        self.read_thread = None
        self.running = False
        self.read_buffer = bytearray()

    def open(self):
        if not self.is_open:
            try:
                self.serial = serial.Serial(self.port, BAUD_RATE, timeout=0.01)
                self.is_open = True
                self.running = True
                self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
                self.read_thread.start()
                self.request_broadcast_ids()
                print(f"Opened serial port: {self.port}")
            except serial.SerialException as e:
                print(f"Failed to open serial port {self.port}: {e}")

    def close(self):
        self.running = False
        self.is_open = False
        if self.serial:
            self.serial.close()

    def read_loop(self):
        while self.running and self.is_open:
            try:
                data = self.serial.read(64)
                if data:
                    self.read_buffer.extend(data)
                    self.process_buffer()
            except serial.SerialException:
                break
            except TypeError:
                break

    def process_buffer(self):
        """Process the read buffer to extract messages."""
        while True:
            # check for delimiter to indicate start of message
            delimiter = self.read_buffer.find(DELIMITER)
            if delimiter == -1:
                return
            if delimiter > 0:
                self.read_buffer = self.read_buffer[delimiter:]

            # wait for 3 more bytes for the header
            if len(self.read_buffer) < len(DELIMITER) + 3:
                return

            # read the header bytes
            header_start = len(DELIMITER)
            pub_id = self.read_buffer[header_start]
            # the "message length" is itself 2 bytes long
            msg_len_bytes = self.read_buffer[header_start + 1:header_start + 3]
            msg_len = msg_len_bytes[1] << 8 | msg_len_bytes[0]

            # wait for full message
            total_length = len(DELIMITER) + 3 + msg_len
            if len(self.read_buffer) < total_length:
                return

            # read in the message bytes
            message_bytes = self.read_buffer[len(DELIMITER) + 3:total_length]
            self.read_buffer = self.read_buffer[total_length:]

            # add the id to the list of publisher ids
            if pub_id not in self.publisher_ids:
                if pub_id not in self.node.ros_pubs and pub_id not in self.node.ros_subs:
                    print(f"Publisher ID {pub_id} not found in ROS publishers or subscribers")
                    return
                self.publisher_ids.append(pub_id)
                self.node.publish_serial_port_state()
                print(f"Added message type id {pub_id}")

            # decode the message with error handling
            if pub_id in self.node.ros_pubs:
                try:
                    bytes_read = self.decode_message(pub_id, message_bytes)
                    if bytes_read != msg_len:
                        print(f"Warning: decoded {bytes_read} bytes but expected {msg_len}")
                except Exception as e:
                    print(f"Failed to decode message for id {pub_id}: {e}")
                    print(message_bytes)
    
    def read_serial_bytes(self, num_bytes: int):
        if not self.is_open: return b''
        try:
            # read the bytes from the serial port
            return self.serial.read(num_bytes)
        except serial.SerialException as e:
            # self.node.remove_serial_port(self.port)
            return b''
    
    def write_serial_bytes(self, data: bytes):
        if not self.is_open: return
        try:
            self.serial.write(data)
        except serial.SerialException as e:
            pass#self.node.remove_serial_port(self.port)
    
    def parse_byte(self, byte: bytes):
        # check for delimiter to indicate start of message
        # print(f"{byte}")
        if (self.previous_byte + byte) != DELIMITER:
            self.previous_byte = byte
            return
        self.previous_byte = byte

        # read 3 more bytes of header
        header_bytes = self.read_serial_bytes(3)
        if len(header_bytes) < 3:
            print(f"Warning: not enough bytes read from serial port {self.port} for the header")
            return
        pub_id_byte = [header_bytes[0]]
        message_length = int.from_bytes([header_bytes[2]]) << 8 | int.from_bytes([header_bytes[1]])
        # print(f"Header bytes: {header_bytes}, pub_id: {pub_id_byte}, message_length: {message_length}")

        # this is the number representing the type of data that will be published by ROS
        # (for example, id of 5 could mean the Motors message)
        pub_id = int.from_bytes(pub_id_byte)
        # record the publisher id
        if pub_id not in self.publisher_ids:
            # check if the pub_id exists
            if pub_id not in self.node.ros_pubs and pub_id not in self.node.ros_subs:
                print(f"Publisher ID {pub_id} not found in ROS publishers or subscribers")
                return
            self.publisher_ids.append(pub_id)
            print(f"Added message type id {pub_id}")
        
        if pub_id in self.node.ros_pubs:
            # read in message_length bytes
            message_bytes = self.read_serial_bytes(message_length)
            # print(f"Read {len(message_bytes)} bytes from serial port {self.port} with id {pub_id}")
            if len(message_bytes) < message_length:
                print(f"Warning: not enough bytes read from serial port {self.port} with id {pub_id} for the message")
                print(message_bytes)
                return
            try:
                bytes_read = self.decode_message(pub_id, message_bytes)
            except Exception as e:
                print(f"Failed to decode message from serial port {self.port} with id {pub_id}: {e}")
                print(message_bytes)
                return
            if bytes_read != message_length:
                print(f"Warning: read {bytes_read} bytes, message claimed to contain {message_length} bytes (id: {pub_id})")

    def decode_message(self, pub_id: int, message: bytes):
        # get ros msg type
        ros_pub: Publisher = self.node.ros_pubs[pub_id]
        ros_msg_type = ros_pub.msg_type
        # deserialize
        ros_msg = deserialize(message, ros_msg_type)
        # if pub_id != 0:
        # print(f"Deserialized message: {ros_msg}")
        # publish to ros
        ros_pub.publish(ros_msg)
        # find size of the message
        serialized_msg: bytes = serialize(ros_msg)
        return len(serialized_msg)
    
    def write_msg_to_serial(self, pub_id: int, msg: any):
        # serialize the msg
        serialized_msg: bytes = serialize(msg)
        
        # write the delimiter
        self.write_serial_bytes(DELIMITER)
        # write the header:
        # write the pub_id
        self.write_serial_bytes(pub_id.to_bytes(1))
        # write the length of the message
        msg_length = len(serialized_msg)
        self.write_serial_bytes(msg_length.to_bytes(2))
        # write the message itself
        self.write_serial_bytes(serialized_msg)
        print(f"Sent message {msg} to serial port {self.port} with id {pub_id} and length {msg_length}")
    
    def request_broadcast_ids(self):
        # write the delimiter
        self.write_serial_bytes(DELIMITER)
        # write the id of a request for a broadcast of ids (0xFF)
        self.write_serial_bytes(b'\xFF')
        print(f"Requesting broadcast of ids from {self.port}")
    
    def request_arduino_upload(self, folder: str, is_nano: bool):
        self.close()
        upload_sketch(self.port, f"~/NULunabotics/Software/ESP8266/{folder}/{folder}.ino", is_nano)
        self.open()
