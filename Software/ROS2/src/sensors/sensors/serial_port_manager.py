from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from sensors.spin_node_helper import spin_nodes
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from lunabotics_interfaces.msg import SerialPortState, SerialPortStates

from sensors.serial import SerialPort, SerialPortObserver, read_id_map_file


# event that tells us when we have a new usb connected
# hires and fires serial port nodes 
# (responsible for connecting publishers and subscribers to serial ports)
class SerialPortManager(Node):
    def __init__(self):
        super().__init__('serial_port_manager')
        self.arduino_upload_sub = self.create_subscription(
            String,
            'website/arduino_upload',
            self.on_arduino_upload,
            10)
        self.serial_port_pub = self.create_publisher(
            SerialPortStates,
            'sensors/serial_port_state',
            10)
        # publisher id to publisher
        self.ros_pubs: dict[int, Publisher] = {}
        # subscriber id to subscriber
        self.ros_subs: dict[int, Subscription] = {}
        # keep track of the serial ports
        self.serial_ports: dict[str, SerialPort] = {}
        # the observer observers the serial ports and calls the callback functions when a port is added or removed
        self.observer = SerialPortObserver(self.add_serial_port, self.remove_serial_port)
        # keep track of arduino folders
        self.arduino_folders: dict[int, str] = {}
        self.is_board_nano: dict[int, bool] = {}
        self.is_board_old_nano: dict[int, bool] = {}
        self.publish_serial_port_state()
    
    def on_arduino_upload(self, msg: String):
        # this is called when a new arduino upload is requested
        # find the serial port
        for port in list(self.serial_ports.keys()):
            if port == msg.data:
                # request the arduino to upload
                ids = self.serial_ports[port].publisher_ids
                if len(ids) == 0:
                    print(f"Serial port {port} has no publisher ids")
                    return
                folder = self.arduino_folders[ids[0]]
                is_nano = self.is_board_nano[ids[0]]
                is_old_nano = self.is_board_old_nano[ids[0]]
                print("uploading arduino folder", folder)
                self.serial_ports[port].request_arduino_upload(folder, is_nano, is_old_nano)
                break
        else:
            print(f"Serial port {msg.data} not found")
    
    def publish_serial_port_state(self):
        # publish the state of the serial ports
        msg = SerialPortStates()
        states = []
        for port in list(self.serial_ports.keys()):
            state = SerialPortState()
            state.port = port
            ids = self.serial_ports[port].publisher_ids
            if len(ids) == 0:
                state.name = "N/A"
            else:
                first_id = ids[0]
                state.name = self.arduino_folders[first_id]
            topics = [self.ros_pubs[i].topic_name for i in ids if i in self.ros_pubs]
            topics += [self.ros_subs[i].topic_name for i in ids if i in self.ros_subs]
            state.topics = topics
            states.append(state)
        msg.states = states
        print(f"Publishing serial port state: {msg}")
        self.serial_port_pub.publish(msg)
    
    def serial_timer_callback(self):
        # check if any serial ports are open
        for port in self.serial_ports.values():
            if not port.is_open: continue
            # read from the serial port
            byte = port.read_serial_bytes(1)
            if byte != b'':
                port.parse_byte(byte) 
    
    def close_serial_ports(self):
        for port, serial_obj in self.serial_ports.items():
            serial_obj.is_open = False
            serial_obj.serial.close()
    
    # add a serial port to the list
    # this is called when the serial port is connected
    def add_serial_port(self, port: str):
        # ignore lidar
        if port == "/dev/ttyUSB1": return
        if port not in self.serial_ports:
            # dependency injection of this node into the serial port
            new_port = SerialPort(self, port)
            new_port.open()
            # add the serial port to the map
            self.serial_ports[port] = new_port
            self.publish_serial_port_state()
            print(f"Added serial port {port}")
        else:
            print(f"Serial port {port} already exists")
    
    # remove a serial port from the list
    # this is called when the serial port is disconnected
    def remove_serial_port(self, port: str):
        if port in self.serial_ports:
            self.serial_ports[port].is_open = False
            # close the serial port
            self.serial_ports[port].serial.close()
            # remove the serial port from the list
            del self.serial_ports[port]
            self.publish_serial_port_state()
            print(f"Removed serial port {port}")
        else:
            print(f"Serial port {port} does not exist")
            
    def on_new_msg(self, pub_id: int, msg: any):
        # this is called when a new message is received from a ros subscription
        # find all serial ports with matching publisher id
        for port in list(self.serial_ports.keys()):
            if pub_id in self.serial_ports[port].publisher_ids:
                # write the message to the serial port
                self.serial_ports[port].write_msg_to_serial(pub_id, msg)
                break
        else:
            print(f"Id {pub_id} not found in any serial ports")
            for port in list(self.serial_ports.values()):
                port.request_broadcast_ids()

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
            new_sub = self.create_subscription(ros_msg_type, topic, partial(self.on_new_msg, pub_id), 10)
            self.ros_subs[pub_id] = new_sub
            print(f"Added ROS subscriber {topic} with ID {pub_id}")
        else:
            print(f"Subscriber ID {pub_id} already exists")


def main(args=None):
    rclpy.init(args=args)
    manager = SerialPortManager()
    # read the id map file from ros resource
    resource_name = 'serial_port_id_map.json'
    resource_path = get_package_share_directory("sensors") + '/' + resource_name
    read_id_map_file(manager, resource_path)
    spin_nodes(manager, shutdown_callback=lambda n: n.close_serial_ports())


if __name__ == '__main__':
    main()
