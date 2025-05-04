import re
import pyudev
from pyudev import Device


class SerialPortObserver:
    """Looks for open serial ports and calls a callback function when any are opened."""
    
    def __init__(self, begin_serial_callback, end_serial_callback):
        self.begin_serial_callback = begin_serial_callback
        self.end_serial_callback = end_serial_callback
        self.monitor_usb_serial()

    def monitor_usb_serial(self):
        """Begins checking for serial ports."""
        context = pyudev.Context()
        monitor = pyudev.Monitor.from_netlink(context)
        # tty is the prefix to serial devices in /dev/ on linux
        monitor.filter_by(subsystem='tty')

        observer = pyudev.MonitorObserver(monitor, callback=self.device_event, name='usb-monitor')
        observer.start()
        
        self.find_existing_ports(context)
    
    def is_usb_serial(self, device):
        """Checks if a device is a USB connection such as /dev/ttyUSB0"""
        # Only match /dev/ttyUSB* or /dev/ttyACM*
        return (
            device.device_node and
            re.match(r'/dev/tty(USB|ACM)\d+', device.device_node) and
            device.get('ID_BUS') == 'usb'
        )


    def device_event(self, device: Device):
        """Handles USB serial device events. Listens for add/remove events."""
        if not self.is_usb_serial(device):
            return

        port = device.device_node
        if device.action == 'add':
            print(f"USB serial device connected: {port}")
            self.begin_serial_callback(port)
        elif device.action == 'remove':
            print(f"USB serial device disconnected: {port}")
            self.end_serial_callback(port)

    def find_existing_ports(self, context: pyudev.Context):
        """Finds existing USB serial devices and starts them."""
        for device in context.list_devices(subsystem='tty'):
            if self.is_usb_serial(device):
                port = device.device_node
                print(f"Found existing USB serial device: {port}")
                self.begin_serial_callback(port)
