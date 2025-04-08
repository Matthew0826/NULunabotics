from lunabotics_interfaces.srv import SerialPort

import rclpy
from rclpy.node import Node

import serial
import serial.tools.list_ports
import os


# https://github.com/giampaolo/psutil/blob/5ba055a8e514698058589d3b615d408767a6e330/psutil/_psposix.py#L28-L53
# https://stackoverflow.com/questions/568271/how-to-check-if-there-exists-a-process-with-a-given-pid-in-python
def pid_exists(pid):
    """Check whether pid exists in the current process table."""
    if pid == 0:
        # According to "man 2 kill" PID 0 has a special meaning:
        # it refers to <<every process in the process group of the
        # calling process>> so we don't want to go any further.
        # If we get here it means this UNIX platform *does* have
        # a process with id 0.
        return True
    try:
        os.kill(pid, 0)
    except OSError as err:
        if err.errno == errno.ESRCH:
            # ESRCH == No such process
            return False
        elif err.errno == errno.EPERM:
            # EPERM clearly means there's a process to deny access to
            return True
        else:
            # According to "man 2 kill" possible error values are
            # (EINVAL, EPERM, ESRCH) therefore we should never get
            # here. If we do let's be explicit in considering this
            # an error.
            raise err
    else:
        return True


class SerialPortService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.ports = {} # keys: port names, values: pids
        self.srv = self.create_service(SerialPort, 'serial_port', self.serial_port_callback)

    def serial_port_callback(self, request, response):
        board_id = request.board_id
        pid = request.pid
        baud_rate = request.baud_rate

        # check if the pid is already assigned a port for whatever reason
        matching_ids = [pid for port, other_pid in self.ports.items() if other_pid == pid]
        if len(matching_ids) > 0:
            response.port = matching_ids[0]
            return response

        # find every port that's still in use
        unavailable_ports = [port for port, other_pid in self.ports.items() if pid_exists(other_pid)]
        # find open ports that aren't in use
        open_ports = [port for port, desc, hwid in serial.tools.list_ports.comports() if port not in unavailable_ports]
        print(f"Open ports: {open_ports}")
        # ask each port which board_id it is
        for port in open_ports:
            try:
                print(f"Trying port {port} with pid {pid} and baud rate {baud_rate}")
                ser = serial.Serial(port=port, baudrate=baud_rate, timeout=2)
                print("Opened port")
                # write header of 0xFFFE
                ser.write(b'\xFF\xFE')
                print("wrote header")
                # read board_id response
                buffer = []
                while len(buffer) < 3 or buffer[0] != 0xFF or buffer[1] != 0xFE:
                    new_byte = ser.read(1)
                    #print(f"new byte: {new_byte}")
                    if new_byte != b'':
                        buffer.append(new_byte[0])
                        #print(f"buffer: {buffer}")
                    buffer = buffer[-3:] # keep only the last 3 bytes

                print("read response")
                port_board_id_byte = buffer[-1]
                print(port_board_id_byte)
                port_board_id = 255 - int.from_bytes(port_board_id_byte, byteorder='big')
                print(f"Port: {port}, Board ID: {port_board_id}, Expected: {board_id}")
                ser.close()
                if port_board_id == board_id:
                    print("Found matching board ID")
                    # port found successfully, sending response
                    self.ports[port] = pid
                    response.port = port
                    return response
            except serial.SerialException as e:
                print(f"Serial exception: {e}")
                continue
        # nothing found, return -1
        response.port = "-1"
        return response


def main():
    rclpy.init()
    serial_port_service = SerialPortService()
    rclpy.spin(serial_port_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
