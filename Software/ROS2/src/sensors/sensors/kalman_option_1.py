import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# THIS HASN'T BEEN TESTED
# IT TREATS THE ACCELEROMETER DATA AS A CONTROL INPUT IN PREDICT
# I think this will work better if the accelerometer data is not noisy and can be treated as truth

# Constants
MARGIN_OF_ERROR = 0.1
STANDARD_DEVIATION = MARGIN_OF_ERROR / 1.96


# Define the Point class
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_distance_with_error(self, other):
        true_dist = ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5
        error = np.random.normal(0, STANDARD_DEVIATION)  # Add random noise
        return true_dist + error

    def get_array(self):
        return np.array([self.x, self.y])


# Define the Extended Kalman Filter class with accelerometer as control input
class ExtendedKalmanFilter:
    def __init__(self, F, B, Q, R, x0, P0, fixed_points):
        self.F = F  # State transition matrix
        self.B = B  # Control input matrix for acceleration
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.x = x0  # Initial state estimate
        self.P = P0  # Initial error covariance
        self.fixed_points = fixed_points  # Fixed points for distance measurements

        # Pre-compute fixed point arrays for efficiency
        self.fixed_point_arrays = np.array([[p.x, p.y] for p in fixed_points])

    def predict(self, acceleration=None):
        if acceleration is not None:
            # State prediction with acceleration as control input
            self.x = self.F @ self.x + self.B @ acceleration
        else:
            # State prediction without control input (fall back to constant velocity)
            self.x = self.F @ self.x

        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q

    def compute_measurement_jacobian(self):
        # Initialize H matrix with zeros
        H = np.zeros((len(self.fixed_points), len(self.x)))

        # Current state position
        x, y = self.x[0], self.x[2]

        # Calculate Jacobian entries for each fixed point
        for i, point in enumerate(self.fixed_points):
            # Distance from current estimated position to fixed point
            dx = x - point.x
            dy = y - point.y
            dist = np.sqrt(dx ** 2 + dy ** 2)

            if dist < 1e-6:  # Avoid division by zero
                dist = 1e-6

            # Derivatives of distance with respect to x, vx, y, vy
            H[i, 0] = dx / dist  # ∂d/∂x
            H[i, 1] = 0  # ∂d/∂vx = 0
            H[i, 2] = dy / dist  # ∂d/∂y
            H[i, 3] = 0  # ∂d/∂vy = 0

        return H

    def compute_expected_measurements(self):
        # Calculate expected distances based on current state
        x, y = self.x[0], self.x[2]

        # Vector of distances from current state to all fixed points
        dx = x - self.fixed_point_arrays[:, 0]
        dy = y - self.fixed_point_arrays[:, 1]
        distances = np.sqrt(dx ** 2 + dy ** 2)

        return distances

    def update(self, measurements):
        # Get measurement Jacobian at current state
        H = self.compute_measurement_jacobian()

        # Calculate expected measurements
        expected_z = self.compute_expected_measurements()

        # Innovation/residual
        y = measurements - expected_z

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain - using direct computation which is safer for this case
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Fallback to pseudoinverse if regular inverse fails
            K = self.P @ H.T @ np.linalg.pinv(S)

        # Update state estimate
        self.x = self.x + K @ y

        # Joseph form for covariance update (more numerically stable)
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R @ K.T

        # Ensure symmetry and positive definiteness
        self.P = (self.P + self.P.T) / 2

        # Add small value to diagonal to ensure positive definiteness
        self.P = self.P + np.eye(len(self.x)) * 1e-6


# Set up fixed reference points
a = Point(292, 0)
b = Point(0, 115)
c = Point(0, 0)
fixed_points = [a, b, c]

# EKF initialization with tuned parameters
dt = 0.1  # Time step

# State transition matrix (x, vx, y, vy)
F = np.array([
    [1, dt, 0, 0],  # x = x + vx*dt
    [0, 1, 0, 0],  # vx = vx
    [0, 0, 1, dt],  # y = y + vy*dt
    [0, 0, 0, 1]  # vy = vy
])

# Control input matrix for acceleration
B = np.array([
    [0.5 * dt ** 2, 0],  # x += 0.5 * ax * dt^2
    [dt, 0],  # vx += ax * dt
    [0, 0.5 * dt ** 2],  # y += 0.5 * ay * dt^2
    [0, dt]  # vy += ay * dt
])

# Process noise covariance matrix
sigma_a = 0.01  # process noise due to acceleration uncertainty
G = np.array([
    [dt ** 2 / 2, 0],
    [dt, 0],
    [0, dt ** 2 / 2],
    [0, dt]
])
Q = G @ np.array([[sigma_a ** 2, 0], [0, sigma_a ** 2]]) @ G.T

# Measurement noise covariance matrix (for distance measurements only)
R = np.eye(3) * (STANDARD_DEVIATION ** 2)

# Initial state estimate - [x, vx, y, vy]
x0 = np.array([5.0, 0.1, 5.0, 0.1])

# Initial uncertainty
P0 = np.diag([0.5, 0.05, 0.5, 0.05])

# Create EKF instance
ekf = ExtendedKalmanFilter(F, B, Q, R, x0, P0, fixed_points)

# Track the EKF estimates over time
MAX_HISTORY = 100
ekf_estimates = deque(maxlen=MAX_HISTORY)
innovation_history = deque(maxlen=MAX_HISTORY)
P_history = deque(maxlen=MAX_HISTORY)


def find_robot_location(da, db, dc, ax=None, ay=None):
    # Process acceleration if provided
    acceleration = np.array([ax, ay]) if ax is not None and ay is not None else None

    # EKF predict step with or without acceleration
    ekf.predict(acceleration)

    # Distance measurements
    measurements = np.array([da, db, dc])

    # Calculate pre-update expected measurements for innovation tracking
    expected_z = ekf.compute_expected_measurements()
    innovation = measurements - expected_z
    innovation_history.append(innovation)

    # Update step with distance measurements
    ekf.update(measurements)
    print(ekf.x)

    # Store current estimate and covariance
    ekf_estimates.append(np.array([ekf.x[0], ekf.x[2]]))  # [x, y]
    P_history.append(np.array([ekf.P[0, 0], ekf.P[2, 2]]))  # Position variances

    # Return the estimated position
    return ekf.x[0], ekf.x[2]