import time

import numpy as np
import matplotlib.pyplot as plt

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


# Define the Extended Kalman Filter class with enhanced tuning
class ExtendedKalmanFilter:
    def __init__(self, F, Q, R, x0, P0, fixed_points):
        self.F = F  # State transition matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.x = x0  # Initial state estimate
        self.P = P0  # Initial error covariance
        self.fixed_points = fixed_points  # Fixed points for distance measurements

        # Pre-compute fixed point arrays for efficiency
        self.fixed_point_arrays = np.array([[p.x, p.y] for p in fixed_points])

    def predict(self):
        # State prediction
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


# Function to simulate the true moving point
def generate_moving_points(start_point, num_steps=50, step_size=0.1):
    points = []
    current_x, current_y = start_point

    for i in range(num_steps):
        # Linear motion along x-axis with slight noise for realism
        new_x = current_x + step_size
        new_y = current_y

        points.append(Point(new_x, new_y))

        # Update current position
        current_x, current_y = new_x, new_y

    return points


# Set up fixed reference points (keep the original three)
a = Point(0, 1)  # Fixed point a
b = Point(0, 0)  # Fixed point b
c = Point(1, 0)  # Fixed point c
fixed_points = [a, b, c]

# EKF initialization with tuned parameters
dt = 0.1  # Smaller time step for better tracking

# State transition matrix (x, vx, y, vy)
F = np.array([
    [1, dt, 0, 0],  # x = x + vx*dt
    [0, 1, 0, 0],  # vx = vx
    [0, 0, 1, dt],  # y = y + vy*dt
    [0, 0, 0, 1]  # vy = vy
])

# Process noise covariance matrix using continuous-time model
sigma_a = 0.01  # process noise due to acceleration uncertainty
G = np.array([
    [dt ** 2 / 2, 0],
    [dt, 0],
    [0, dt ** 2 / 2],
    [0, dt]
])
Q = G @ np.array([[sigma_a ** 2, 0], [0, sigma_a ** 2]]) @ G.T

# Measurement noise - slightly higher to account for distance-based errors
R = np.eye(3) * (STANDARD_DEVIATION ** 2)

# Initial state estimate - better to start slightly off than grossly wrong
# [x, vx, y, vy]
x0 = np.array([4.8, 0.1, 4.0, 0.0])  # Close to true starting position with some velocity

# Initial uncertainty - smaller values to indicate more confidence in initial state
P0 = np.diag([0.5, 0.05, 0.5, 0.05])

# Create EKF instance
ekf = ExtendedKalmanFilter(F, Q, R, x0, P0, fixed_points)

# Generate true trajectory (starting at [5, 4])
true_points = generate_moving_points([5, 4], num_steps=100, step_size=0.05)

# Track the EKF estimates over time
ekf_estimates = []
innovation_history = []
P_history = []

for step, true_point in enumerate(true_points):
    # Get distance measurements from fixed points (with noise)
    measurements = np.array([
        point.get_distance_with_error(true_point) for point in fixed_points
    ])
    start = time.time()
    # EKF predict and update steps
    ekf.predict()

    # Calculate pre-update expected measurements for innovation tracking
    expected_z = ekf.compute_expected_measurements()
    innovation = measurements - expected_z
    innovation_history.append(innovation)

    # print(measurements)
    # Update step
    ekf.update(measurements)
    end = time.time()

    print(end)
    print(start)
    print(ekf.x)

    # Store current estimate and covariance
    ekf_estimates.append(np.array([ekf.x[0], ekf.x[2]]))  # [x, y]
    P_history.append(np.array([ekf.P[0, 0], ekf.P[2, 2]]))  # Position variances

# Convert to numpy arrays for plotting
ekf_estimates = np.array(ekf_estimates)
true_points_array = np.array([[p.x, p.y] for p in true_points])
P_history = np.array(P_history)
innovation_history = np.array(innovation_history)

# Calculate performance metrics
position_errors = np.sqrt(np.sum((ekf_estimates - true_points_array) ** 2, axis=1))
rmse = np.sqrt(np.mean(position_errors ** 2))
max_error = np.max(position_errors)
avg_error = np.mean(position_errors)

# Plotting
plt.figure(figsize=(15, 10))

# Main trajectory plot
plt.subplot(2, 2, 1)
for i, point in enumerate(fixed_points):
    plt.scatter(point.x, point.y, color='red', s=100, marker='^')
    plt.annotate(f"{chr(97 + i)}", (point.x, point.y), xytext=(5, 5), textcoords='offset points')

plt.plot(true_points_array[:, 0], true_points_array[:, 1], 'g-', linewidth=2, label='True Trajectory')
plt.plot(ekf_estimates[:, 0], ekf_estimates[:, 1], 'b--', linewidth=2, label='EKF Estimate')
plt.scatter(true_points_array[0, 0], true_points_array[0, 1], color='green', s=100, marker='o', label='Start')
plt.scatter(true_points_array[-1, 0], true_points_array[-1, 1], color='green', s=100, marker='s', label='End')
plt.scatter(ekf_estimates[0, 0], ekf_estimates[0, 1], color='blue', s=100, marker='o')
plt.scatter(ekf_estimates[-1, 0], ekf_estimates[-1, 1], color='blue', s=100, marker='s')

# Draw uncertainty ellipses at regular intervals
for i in range(0, len(ekf_estimates), 10):
    if i < len(P_history):
        std_x = np.sqrt(P_history[i, 0])
        std_y = np.sqrt(P_history[i, 1])

        # Create ellipse points
        theta = np.linspace(0, 2 * np.pi, 100)
        ellipse_x = ekf_estimates[i, 0] + 2 * std_x * np.cos(theta)  # 2-sigma ellipse
        ellipse_y = ekf_estimates[i, 1] + 2 * std_y * np.sin(theta)
        plt.plot(ellipse_x, ellipse_y, 'b:', alpha=0.3)

plt.grid(True)
plt.legend()
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Extended Kalman Filter with 3 Fixed Points')

# Error over time plot
plt.subplot(2, 2, 2)
plt.plot(position_errors, 'r-')
plt.axhline(y=rmse, color='k', linestyle='--', label=f'RMSE: {rmse:.4f}')
plt.grid(True)
plt.xlabel('Time Step')
plt.ylabel('Position Error')
plt.title('EKF Estimation Error Over Time')
plt.legend()

# X and Y estimates over time
plt.subplot(2, 2, 3)
plt.plot(true_points_array[:, 0], 'g-', label='True X')
plt.plot(ekf_estimates[:, 0], 'b--', label='Estimated X')
plt.fill_between(range(len(ekf_estimates)),
                 ekf_estimates[:, 0] - 2 * np.sqrt(P_history[:, 0]),
                 ekf_estimates[:, 0] + 2 * np.sqrt(P_history[:, 0]),
                 color='blue', alpha=0.2)
plt.grid(True)
plt.xlabel('Time Step')
plt.ylabel('X Position')
plt.title('X Position Tracking')
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(true_points_array[:, 1], 'g-', label='True Y')
plt.plot(ekf_estimates[:, 1], 'b--', label='Estimated Y')
plt.fill_between(range(len(ekf_estimates)),
                 ekf_estimates[:, 1] - 2 * np.sqrt(P_history[:, 1]),
                 ekf_estimates[:, 1] + 2 * np.sqrt(P_history[:, 1]),
                 color='blue', alpha=0.2)
plt.grid(True)
plt.xlabel('Time Step')
plt.ylabel('Y Position')
plt.title('Y Position Tracking')
plt.legend()

plt.tight_layout()

# Print performance metrics
print(f"Root Mean Square Error: {rmse:.4f}")
print(f"Maximum Error: {max_error:.4f}")
print(f"Average Error: {avg_error:.4f}")

plt.show()

# Extra analysis: Plot innovation sequence
plt.figure(figsize=(10, 6))
for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.plot(innovation_history[:, i])
    plt.axhline(y=0, color='r', linestyle='-')
    plt.axhline(y=2 * STANDARD_DEVIATION, color='r', linestyle='--')
    plt.axhline(y=-2 * STANDARD_DEVIATION, color='r', linestyle='--')
    plt.title(f'Innovation Sequence for Measurement {i + 1}')
    plt.grid(True)
    plt.ylabel('Innovation')
    if i == 2:
        plt.xlabel('Time Step')
plt.tight_layout()
plt.show()