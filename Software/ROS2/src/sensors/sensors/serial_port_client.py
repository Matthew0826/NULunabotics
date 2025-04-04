import rclpy
import time
from rclpy.node import Node
from lunabotics_interfaces.srv import SerialPort


class SerialPortClient(Node):
    def __init__(self):
        super().__init__('serial_port_client')
        self.cli = self.create_client(SerialPort, 'serial_port')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SerialPort.Request()

    def send_request(self, board_id, pid, baud_rate):
        self.req.board_id = board_id
        self.req.pid = pid
        self.req.baud_rate = baud_rate
        return self.cli.call_async(self.req)

def find_port(esp_board_id: int, pid: int, baud_rate: int) -> str:
    serial_port_client = SerialPortClient()
    port = "-1"
    while port == "-1":
        future = serial_port_client.send_request(esp_board_id, pid, baud_rate)
        rclpy.spin_until_future_complete(serial_port_client, future)
        response = future.result()
        port = response.port
        if port == "-1":
            serial_port_client.get_logger().info("No available ports found, trying again in 5 seconds.")
            time.sleep(5)
    serial_port_client.destroy_node()
