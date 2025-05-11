import numpy as np
from collections import deque

# THIS HASN'T BEEN TESTED
# IT TREATS THE ACCELEROMETER DATA AS AN ADDITIONAL MEASUREMENT ALONG WITH THE BEACON DISTANCES
# I think this means it will work better if the accelerometer data is noisy

# Constants
MARGIN_OF_ERROR = 0.1
STANDARD_DEVIATION = MARGIN_OF_ERROR / 1.96
ACCEL_STANDARD_DEVIATION = 0.05  # Standard deviation for accelerometer measurements


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


# Define the Extended Kalman Filter class with accelerometer as measurement
class ExtendedKalmanFilter:
    def __init__(self, F, Q, R, x0, P0, fixed_points):
        self.F = F  # State transition matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance (includes both distance and accel measurements)
        self.x = x0  # Initial state estimate [x, vx, y, vy]
        self.P = P0  # Initial error covariance
        self.fixed_points = fixed_points  # Fixed points for distance measurements

        # Pre-compute fixed point arrays for efficiency
        self.fixed_point_arrays = np.array([[p.x, p.y] for p in fixed_points])

        # Number of fixed points (for measurement vector sizing)
        self.num_fixed_points = len(fixed_points)

    def predict(self):
        # Standard predict step (no control input)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def compute_measurement_jacobian(self, with_accel=False):
        # Calculate how many measurements we have (distances + accelerometer)
        num_measurements = self.num_fixed_points
        if with_accel:
            num_measurements += 2  # Add two more for ax and ay

        # Initialize H matrix with zeros
        H = np.zeros((num_measurements, len(self.x)))

        # Current state position
        x, y = self.x[0], self.x[2]

        # Calculate Jacobian entries for each fixed point (distance measurements)
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

        # If we're including accelerometer measurements
        if with_accel:
            # The accelerometer measures the derivative of velocity
            # For x acceleration: ax ≈ (vx_new - vx_old)/dt
            # Since vx_old = x[1] and vx_new ≈ vx_old (in the limit as dt→0)
            # We can approximate ax ≈ 0 for constant velocity

            # Row for ax measurement: We expect ax = 0 (constant velocity)
            # Jacobian is 0 for all state variables
            H[self.num_fixed_points, 1] = 1.0 / self.F[1, 1]  # Relates vx to ax

            # Row for ay measurement: We expect ay = 0 (constant velocity)
            # Jacobian is 0 for all state variables
            H[self.num_fixed_points + 1, 3] = 1.0 / self.F[3, 3]  # Relates vy to ay

        return H

    def compute_expected_measurements(self, with_accel=False):
        # Calculate expected distances based on current state
        x, y = self.x[0], self.x[2]

        # Vector of distances from current state to all fixed points
        dx = x - self.fixed_point_arrays[:, 0]
        dy = y - self.fixed_point_arrays[:, 1]
        distances = np.sqrt(dx ** 2 + dy ** 2)

        # If we're including accelerometer measurements
        if with_accel:
            # In a constant velocity model, expected acceleration is zero
            expected_ax = 0.0
            expected_ay = 0.0

            # Combine distance and acceleration measurements
            return np.concatenate([distances, [expected_ax, expected_ay]])

        return distances

    def update(self, distance_measurements, accel_measurements=None):
        # Determine if we have accelerometer measurements
        with_accel = accel_measurements is not None

        # Get measurement Jacobian at current state
        H = self.compute_measurement_jacobian(with_accel)

        # Calculate expected measurements
        expected_z = self.compute_expected_measurements(with_accel)

        # Combine distance measurements with accelerometer measurements if available
        if with_accel:
            measurements = np.concatenate([distance_measurements, accel_measurements])
        else:
            measurements = distance_measurements

        # Innovation/residual
        y = measurements - expected_z

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
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
        self.P = self.P + np.eye(len(self.x)) * 1e-6


# Set up fixed reference points
a = Point(0, 0)
b = Point(0, 200)
c = Point(200, 0)
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

# Process noise covariance matrix
sigma_a = 0.01  # process noise due to acceleration uncertainty
G = np.array([
    [dt ** 2 / 2, 0],
    [dt, 0],
    [0, dt ** 2 / 2],
    [0, dt]
])
Q = G @ np.array([[sigma_a ** 2, 0], [0, sigma_a ** 2]]) @ G.T

# Initial state estimate - [x, vx, y, vy]
x0 = np.array([20.0, 0.1, 20.0, 0.1])

# Initial uncertainty
P0 = np.diag([0.5, 0.05, 0.5, 0.05])

# Measurement noise covariance - need to create a larger matrix to include accelerometer
# First 3 diagonal elements for distance measurement noise
# Last 2 diagonal elements for accelerometer measurement noise
R_distances = np.eye(3) * (STANDARD_DEVIATION ** 2)
R_with_accel = np.zeros((3,3))#(5, 5))  # 3 distances + 2 accelerations
R_with_accel[:3, :3] = R_distances
R_with_accel[3:, 3:] = np.eye(2) * (ACCEL_STANDARD_DEVIATION ** 2)

# Create EKF instance
ekf = ExtendedKalmanFilter(F, Q, R_with_accel, x0, P0, fixed_points)

# Track the EKF estimates over time
MAX_HISTORY = 100
ekf_estimates = deque(maxlen=MAX_HISTORY)
innovation_history = deque(maxlen=MAX_HISTORY)
P_history = deque(maxlen=MAX_HISTORY)


def find_robot_location(da, db, dc, ax=None, ay=None):
    # Create measurement vectors
    distance_measurements = np.array([da, db, dc])

    # Check if accelerometer data is provided
    accel_measurements = np.array([ax, ay]) if ax is not None and ay is not None else None

    # EKF predict step
    ekf.predict()

    # Calculate pre-update expected measurements for innovation tracking
    expected_z = ekf.compute_expected_measurements(accel_measurements is not None)

    # Combine measurements if accelerometer data is available
    if accel_measurements is not None:
        measurements = np.concatenate([distance_measurements, accel_measurements])
    else:
        measurements = distance_measurements

    innovation = measurements - expected_z
    innovation_history.append(innovation)

    # Update step with appropriate measurements
    ekf.update(distance_measurements, accel_measurements)
    print(ekf.x)

    # Store current estimate and covariance
    ekf_estimates.append(np.array([ekf.x[0], ekf.x[2]]))  # [x, y]
    P_history.append(np.array([ekf.P[0, 0], ekf.P[2, 2]]))  # Position variances

    # Return the estimated position
    return ekf.x[0], ekf.x[2]