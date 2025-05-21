#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lunabotics_interfaces.msg import PowerData
from sensor_msgs.msg import BatteryState
import numpy as np
from collections import deque

from sensors.spin_node_helper import spin_nodes

class BatteryPercentageCalculator(Node):
    def __init__(self):
        super().__init__('battery_percentage_calculator')
        
        # Battery specifications
        self.nominal_voltage = 12.8  # V
        self.nominal_capacity = 7.0  # Ah
        self.discharge_cutoff = 8.8  # V
        self.reconnect_voltage = 10.8  # V
        self.max_discharge_current = 7.0  # A
        
        # Subscribers
        self.power_sub = self.create_subscription(
            PowerData, 
            'sensors/power_data', 
            self.power_callback, 
            10)
        
        # Publisher
        self.battery_pub = self.create_publisher(
            BatteryState, 
            '/battery/state', 
            10)
        
        # Parameters
        self.declare_parameter('filter_window_size', 10)
        self.filter_window_size = self.get_parameter('filter_window_size').value
        
        # State variables
        self.latest_voltage = None
        self.latest_current = None
        self.voltage_history = deque(maxlen=self.filter_window_size)
        self.current_history = deque(maxlen=self.filter_window_size)
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.update_battery_state)
    
    def power_callback(self, msg):
        volts = msg.voltage12v
        amps = msg.current
        self.latest_voltage = volts
        self.voltage_history.append(volts)
        self.latest_current = amps
        self.current_history.append(amps)
    
    def battery_voltage(self, capacity_percent):
        x = capacity_percent / 100
        return 13.0366 - 1.5192*x - 3.9305*x**2 + 18.0276*x**3 - 15.6828*x**4
    
    def voltage_to_capacity(self, voltage):
        """
        Convert voltage to capacity percentage using the lookup table
        with linear interpolation between points
        """
        if voltage >= 13.0366:
            return 0.0
        if voltage <= 10.0:
            return 100.0
        
        left, right = 0.0, 100.0
        while right - left > 0.01:
            mid = (left + right) / 2
            mid_voltage = self.battery_voltage(mid)
            if mid_voltage > voltage:
                left = mid
            else:
                right = mid
        
        return (left + right) / 2
    
    def correct_for_load(self, voltage, current):
        """
        Correct voltage reading accounting for the load.
        
        At higher loads (current draw), voltage sags more than actual SoC would indicate.
        This function compensates for that effect.
        """
        # Calculate C-rate
        c_rate = abs(current) / self.nominal_capacity
        
        # LFP batteries typically have low internal resistance
        # For a 12.8V, 7Ah LFP battery, a typical value might be around 10-20 milliohms
        # This is an approximation - ideally you'd measure this for your specific battery
        internal_resistance = 0.015  # ohms
        
        # Basic correction: add back the voltage drop due to internal resistance
        corrected_voltage = voltage
        
        if current < 0:  # Discharging (negative current in ROS convention)
            # Add compensation for internal resistance voltage drop
            corrected_voltage = voltage + (abs(current) * internal_resistance)
            
            # Additional correction for high C-rates (if significantly different from 0.5C)
            # This is a simplified model that increases correction as C-rate gets higher
            if c_rate > 0.6:  # If significantly higher than the 0.5C curve
                c_rate_factor = 1.0 + (c_rate - 0.5) * 0.1  # 10% adjustment per 1C above 0.5C
                corrected_voltage *= c_rate_factor
        
        return corrected_voltage
    
    def get_filtered_values(self):
        """Get filtered voltage and current to reduce noise"""
        if not self.voltage_history or not self.current_history:
            return None, None
        
        # Use median filtering to reduce outliers
        filtered_voltage = np.median(list(self.voltage_history))
        filtered_current = np.median(list(self.current_history))
        
        return filtered_voltage, filtered_current
    
    def update_battery_state(self):
        if self.latest_voltage is None or self.latest_current is None:
            self.get_logger().warning('No voltage or current data received yet')
            return
        
        # Get filtered values
        voltage, current = self.get_filtered_values()
        if voltage is None:
            return
            
        # Correct voltage for load effects
        corrected_voltage = self.correct_for_load(voltage, current)
        
        # Calculate capacity percentage
        percentage = 100 - self.voltage_to_capacity(corrected_voltage)
        
        # Calculate C-rate for information
        c_rate = abs(current) / self.nominal_capacity
        
        # Create and publish battery state message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.current = float(current)
        msg.percentage = float(percentage) / 100.0  # Convert to 0-1 range as per ROS convention
        msg.present = True
        
        # Set power supply status
        if current < 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif current > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        
        # Set health status based on voltage
        if voltage < self.discharge_cutoff + 0.2:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif percentage < 10:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT  # Reusing to indicate low
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        
        # Calculate remaining capacity in Ah
        msg.capacity = self.nominal_capacity  # Ah
        msg.design_capacity = self.nominal_capacity  # Ah
        msg.charge = self.nominal_capacity * (percentage / 100.0)  # Ah
        
        self.battery_pub.publish(msg)
        # self.get_logger().info(f'Battery: {percentage:.1f}%, {voltage:.2f}V, {current:.2f}A, C-rate: {c_rate:.2f}C')

def main(args=None):
    rclpy.init(args=args)
    spin_nodes(BatteryPercentageCalculator())

if __name__ == '__main__':
    main()