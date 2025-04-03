import numpy as np
from scipy.optimize import least_squares
import time

MARGIN_OF_ERROR = 0.1
STANDARD_DEVIATION = MARGIN_OF_ERROR / 1.96

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        # self.z = z

    def get_distance_with_error(self, other):
        true_dist = ((self.x - other.x)**2 + (self.y - other.y)**2)**0.5
        distance = np.random.normal(0, STANDARD_DEVIATION)
        # distance = 0
        return true_dist + distance

    def get_array(self):
        return np.array([self.x, self.y])


a = Point(100, 0)
b = Point(100, 100)
c = Point(0, 100)

# Function to minimize
def trilateration(x, da, db, dc):
    return [
        np.linalg.norm(x - a.get_array()) - da,
        np.linalg.norm(x - b.get_array()) - db,
        np.linalg.norm(x - c.get_array()) - dc
    ]

def find_robot_location(da, db, dc):
    # Initial guess for the coordinates of the new point
    initial_guess = Point((a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3)

    # Solve the system of equations
    result = least_squares(trilateration, initial_guess.get_array(), args=(da, db, dc))

    # Resulting coordinates of the new point
    new_point = result.x
    return new_point
