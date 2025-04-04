import rclpy
from rclpy.node import Node
import serial
import json
from scipy.optimize import least_squares
import numpy as np

from geometry_msgs.msg import Point as PointMsg

port = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=1)

# assume input is a json as a list of distances where each item is the distance to that beacon

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        # self.z = z

    # def get_distance_with_error(self, other):
    #     true_dist = ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5
    #     distance = np.random.normal(0, STANDARD_DEVIATION)
    #     # distance = 0
    #     return true_dist + distance

    def get_array(self):
        return np.array([self.x, self.y])
    

NUM_BEACONS = 3
BEACON_LOCATIONS = [Point(0, 1), Point(0, 0), Point(1, 0)]
# Initial guess for the coordinates of the new point
INITIAL_GUESS = Point(3, 3)

class LocationPublisher(Node):
  def __init__(self):
    super().__init__('location_publisher')
    timer_period = 0.02  # 20 ms
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.distances = [None] * NUM_BEACONS

  def timer_callback(self):
    msg = PointMsg()
    if port.in_waiting > 0:
      read_data = port.readlines()
      if len(read_data) > 0:
        json_data = read_data[-1]
      distances = json.loads(json_data)
      self.distances = distances
    
    # Solve the system of equations
    result = least_squares(self.trilateration, INITIAL_GUESS.get_array())
    msg.x = result[0]
    msg.y = result[1]
    msg.z = 0.0
    self.publisher_.publish(msg)

  # Function to minimize
  def trilateration(self, x):
      return [np.linalg.norm(x - beacon_point.get_array()) - distance for beacon_point, distance in zip(BEACON_LOCATIONS, self.distances)]
      
    
    