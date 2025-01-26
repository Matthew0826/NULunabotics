import numpy as np
from scipy.optimize import least_squares

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

a = Point(0, 1)
b = Point(0, 0)
c = Point(1, 0)

l = Point(5, 4)

# Distances to the new point
da = a.get_distance_with_error(l)
db = b.get_distance_with_error(l)
dc = c.get_distance_with_error(l)

# Function to minimize
def trilateration(x):
    return [
        np.linalg.norm(x - a.get_array()) - da,
        np.linalg.norm(x - b.get_array()) - db,
        np.linalg.norm(x - c.get_array()) - dc
    ]

# Initial guess for the coordinates of the new point
initial_guess = Point(3, 3)

# Solve the system of equations
result = least_squares(trilateration, initial_guess.get_array())

# Resulting coordinates of the new point
new_point = result.x
print(f"Coordinates of the new point: {new_point}")
