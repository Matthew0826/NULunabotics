import rclpy
from rclpy.node import Node

from lunabotics_interfaces.msg import Excavator
from sensors.spin_node_helper import spin_nodes

import serial

BAUD_RATE = 9600

# ——— Channel assignments ———
# linear actuator
ELEVATOR_CH  = 4
# conveyor belt
CONVEYOR_CH  = 5
# outtake motor
OUTTAKE_CH   = 6

TOLERANCE = 0.05    # deadband; avoid flooding serial on small jitters

class ExcavatorDriver(Node):
    def __init__(self, port: str):
        super().__init__('excavator_driver')
        self.get_logger().info("Waiting for excavator commands…")
        self.sub = self.create_subscription(
            Excavator,
            'physical_robot/excavator',
            self.listener_callback,
            10
        )

        # connect le serial link to our ESP board
        self.ser = serial.Serial(port, BAUD_RATE, timeout=1)

        # keep track of last‐sent values so we only resend on big changes
        self.prev_elev = 0.0
        self.prev_conv = 0.0
        self.prev_outtake = 0.0

    def send_serial_packet(self, channel: int, value: float):
        """Map –1..+1 into 0..255 and send [0xFF,0xFF, channel, data]"""
        # clamp just in case
        v = max(-1.0, min(1.0, value))
        byte_val = int((v + 1.0) * 127.5)  # –1→0, 0→127.5, +1→255
        packet = b'\xFF\xFF' + channel.to_bytes(1, 'little') + byte_val.to_bytes(1, 'little')
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def listener_callback(self, msg: Excavator):
        # extract desired values
        elev, conv, out = msg.elevator, msg.conveyor, msg.outtake

        # apply deadband (too small to care)
        if abs(elev) < TOLERANCE: elev = 0.0
        if abs(conv) < TOLERANCE: conv = 0.0
        if abs(out) < TOLERANCE: out = 0.0

        # only send if it changed more than tolerance
        if abs(elev - self.prev_elev) > TOLERANCE:
            self.send_serial_packet(ELEVATOR_CH, elev)
            self.prev_elev = elev

        if abs(conv - self.prev_conv) > TOLERANCE:
            self.send_serial_packet(CONVEYOR_CH, conv)
            self.prev_conv = conv

        if abs(out - self.prev_outtake) > TOLERANCE:
            self.send_serial_packet(OUTTAKE_CH, out)
            self.prev_outtake = out


def main(args=None):
    rclpy.init(args=args)
    spin_nodes( ExcavatorDriver("/dev/ttyUSB0") )


if __name__ == '__main__':
    main()