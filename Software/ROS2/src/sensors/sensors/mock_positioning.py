import rclpy
import math
from rclpy.node import Node
from collections import deque

from std_msgs.msg import Float32, Bool

from sensors.spin_node_helper import spin_nodes

from lunabotics_interfaces.msg import Point, Excavator, Motors, ExcavatorPotentiometer, AccelerometerCorrection

import random
import numpy as np

TOP_SPEED = 100.0 # cm/s
REFRESH_RATE = 10 # Hz

class SpacialDataPublisher(Node):

    def __init__(self):
        super().__init__('positioning')
        
        # create position publisher
        self.position_pub = self.create_publisher(Point, '/sensors/position', 10)
        
        # create orientation publisher
        self.orientation_pub = self.create_publisher(Float32, '/sensors/orientation', 10)
        
        # create excavator potentiometer publisher
        self.excavator_potentiometer_pub = self.create_publisher(ExcavatorPotentiometer, '/sensors/excavator_percent', 10)
        
        # listen for resets coming from the website
        self.reset_sub = self.create_subscription(
            Bool,
            "website/reset",
            self.on_reset,
            10)
 
        self.motor_sub = self.create_subscription(
                Motors,
                "physical_robot/motors",
                self.on_motor,
                10)
        self.excavator_sub = self.create_subscription(
                Excavator,
                "physical_robot/excavator",
                self.on_excavator,
                10)
        
        self.orientation_correction_sub = self.create_subscription(AccelerometerCorrection, '/sensors/accelerometer_correction', self.on_orientation_correction, 10)
        
        # keep track of positions
        self.x = 448.0
        self.y = 100.0
        self.orientation = 255.0#float(random.randint(0, 360))
        self.initial_orientation = 0.0# -self.orientation
        
        # to cause a delay in the simulation to mirror real kalman filter delay
        self.position_history_length = 5
        self.position_history = deque(maxlen=self.position_history_length)
        
        self.motor_power_left = 0.0
        self.motor_power_right = 0.0
        
        # keep track of excavator state
        self.actuator_powers = (0.0, 0.0)
        self.actuator_percents = (0.0, 0.0)
        
        self.timer = self.create_timer(1/REFRESH_RATE, self.timer_callback)
    
    def on_motor(self, motors: Motors):
        self.motor_power_left = motors.front_left_wheel
        self.motor_power_right = motors.front_right_wheel
        # self.get_logger().info(f"got motors: {motors.front_left_wheel}, {motors.front_right_wheel}")
    
    def on_reset(self, msg: Bool):
        self.get_logger().info("Resetting position")
        self.x = 448.0
        self.y = 100.0
        self.orientation = float(random.randint(0, 360))
        self.initial_orientation = -self.orientation
        
        self.motor_power_left = 0.0
        self.motor_power_right = 0.0
        
        # keep track of excavator state
        self.actuator_powers = (0.0, 0.0)
        self.actuator_percents = (0.0, 0.0)
    
    def on_excavator(self, excavator: Excavator):
        self.actuator_powers = (excavator.excavator_lifter_speed, excavator.actuator_speed)
    
    def update_simulation(self):
        if abs(self.motor_power_left) > 0.1 or abs(self.motor_power_right) > 0.1:
            # Calculate average motor power for forward/backward movement
            average_power = (self.motor_power_left + self.motor_power_right) / 2.0
            
            # Calculate turning based on difference between motors
            motor_power_delta = self.motor_power_right - self.motor_power_left
            
            tick_speed = TOP_SPEED / REFRESH_RATE
            
            # Update orientation based on motor difference
            self.orientation += motor_power_delta * tick_speed/4.0 + 0.1
            self.orientation = float(self.orientation % 360)
            
            # Move forward/backward based on average power with the corrected orientation
            self.x += math.cos(math.radians(90 - self.orientation)) * average_power * tick_speed
            self.y += math.sin(math.radians(90 - self.orientation)) * average_power * tick_speed
        if abs(self.actuator_powers[0]) > 0.1 or abs(self.actuator_powers[1]) > 0.1:
            new_percent_left = clamp(self.actuator_percents[0] + self.actuator_powers[0] * np.random.uniform(0.04, 0.06))
            new_percent_right = clamp(self.actuator_percents[1] + self.actuator_powers[1] * np.random.uniform(0.04, 0.06))
            self.actuator_percents = (new_percent_left, new_percent_right)
    
    def on_orientation_correction(self, msg: AccelerometerCorrection):
        # Apply the correction to the orientation
        self.get_logger().info(f"Got orientation correction: {msg.initial_angle} (correct was {self.initial_orientation})")
        self.initial_orientation -= msg.initial_angle
        self.get_logger().info(f"New initial orientation: {self.initial_orientation}")
        if msg.should_reset == 1:
            self.orientation = 0.0
            
    def timer_callback(self):
        self.update_simulation()
        msg = Float32()
        msg.data = float(360.0 - ((self.orientation + self.initial_orientation + 360.0) % 360.0))
        # publish mock orientation based on motor power
        self.orientation_pub.publish(msg)
        # self.get_logger().info(f"Publishing orientation: {msg.data}")
        
        msg = Point()
        msg.x = float(self.x)
        msg.y = float(self.y)
        # publish mock position based on motor power
        # but with a delay
        self.position_history.append(msg)
        self.position_pub.publish(self.position_history[0])
        
        msg = ExcavatorPotentiometer()
        msg.excavator_lifter_percent = float(self.actuator_percents[0])
        msg.actuator_percent = float(self.actuator_percents[1])
        # publish mock excavator potentiometer based on motor power
        self.excavator_potentiometer_pub.publish(msg)


def clamp(value, min_value=0.0, max_value=1.0):
    return max(min(value, max_value), min_value)

def main(args=None):
    rclpy.init(args=args)
    spin_nodes(SpacialDataPublisher())


if __name__ == '__main__':
    main()
