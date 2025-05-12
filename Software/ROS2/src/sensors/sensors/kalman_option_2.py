import numpy as np
from collections import deque

# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

# Measurement error parameters
MARGIN_OF_ERROR = 0.1  # Margin of error for distance measurements
STANDARD_DEVIATION = MARGIN_OF_ERROR / 1.96  # Standard deviation derived from margin of error
ACCEL_STANDARD_DEVIATION = 0.05  # Standard deviation for accelerometer measurements

# Validation thresholds
MIN_VALID_DISTANCE = 1.0  # Minimum valid distance measurement (cm)
MAX_VALID_DISTANCE = 15000  # Maximum valid distance measurement (cm)
MAX_VALID_ACCEL = 10.0  # Maximum valid acceleration (m/s²)

# Filter parameters
INNOVATION_THRESHOLD = 5.0  # Threshold for innovation rejection
ACCEL_UNCERTAINTY_BASE = 0.01  # Base process noise for acceleration
ACCEL_UNCERTAINTY_SCALE = 0.05  # Scaling factor for acceleration-based process noise
FILTER_DIVERGENCE_POSITION_VAR_THRESHOLD = 100.0  # Position variance threshold for filter divergence detection
FILTER_DIVERGENCE_INNOVATION_THRESHOLD = 50.0  # Innovation threshold for filter divergence detection
FILTER_MINIMUM_INNOVATION_HISTORY = 10  # Minimum number of samples needed for divergence check
COVARIANCE_RESET_POSITION = 5.0  # Position variance for covariance reset
COVARIANCE_RESET_VELOCITY = 0.5  # Velocity variance for covariance reset

# Numerical stability parameters
MINIMUM_DISTANCE = 1e-6  # Minimum distance to avoid division by zero
COVARIANCE_STABILIZER = 1e-6  # Small value added to ensure positive definiteness

# Adaptive measurement parameters
DISTANCE_SCALING_THRESHOLD = 1000.0  # Distance threshold for scaling measurement uncertainty
ACCEL_SCALING_THRESHOLD = 2.0  # Acceleration threshold for scaling measurement uncertainty

# Low-pass filter parameters
ACCEL_FILTER_ALPHA = 0.3  # Low-pass filter coefficient (0 < alpha < 1)

# Data history
MAX_HISTORY_SIZE = 100  # Maximum size of history deques

# EKF initialization
INIT_X = 50.0  # Initial x position
INIT_VX = 0.1  # Initial x velocity
INIT_Y = 5.0  # Initial y position
INIT_VY = 0.1  # Initial y velocity
INIT_POS_VARIANCE = 0.5  # Initial position variance
INIT_VEL_VARIANCE = 0.05  # Initial velocity variance

# Motion model
TIME_STEP = 0.1  # Time step (seconds)

# Beacon positions
BEACON_A_X = 200
BEACON_A_Y = 0

BEACON_B_X = 0
BEACON_B_Y = 0

BEACON_C_X = 0
BEACON_C_Y = 200


# =============================================================================
# CLASSES AND FUNCTIONS
# =============================================================================

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


# Define the Extended Kalman Filter class with accelerometer and beacons as measurement
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

            if dist < MINIMUM_DISTANCE:  # Avoid division by zero
                dist = MINIMUM_DISTANCE

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
        # Validate measurements first
        valid_distances, valid_accel = validate_measurements(distance_measurements, accel_measurements)
        
        # Determine if we have accelerometer measurements
        with_accel = valid_accel is not None
        
        # Calculate adaptive measurement covariance
        adaptive_R = adaptive_measurement_covariance(valid_distances, valid_accel)
        self.R = adaptive_R  # Update the R matrix dynamically
        
        # Get measurement Jacobian at current state
        H = self.compute_measurement_jacobian(with_accel)
        
        # Calculate expected measurements
        expected_z = self.compute_expected_measurements(with_accel)
        
        # Combine distance measurements with accelerometer measurements if available
        if with_accel:
            measurements = np.concatenate([valid_distances, valid_accel])
        else:
            measurements = valid_distances
        
        # Innovation/residual
        y = measurements - expected_z
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Mahalanobis distance for innovation check
        try:
            mahalanobis_dist = np.sqrt(y.T @ np.linalg.inv(S) @ y)
            
            # If innovation is too large, reduce the Kalman gain
            if mahalanobis_dist > INNOVATION_THRESHOLD:
                scale_factor = INNOVATION_THRESHOLD / mahalanobis_dist
                print(f"Large innovation detected (d={mahalanobis_dist:.2f}), scaling down influence")
        except np.linalg.LinAlgError:
            # If inversion fails, continue without innovation check
            mahalanobis_dist = float('inf')
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
            
            # Scale down Kalman gain if innovation is too large
            if mahalanobis_dist > INNOVATION_THRESHOLD:
                K = K * (INNOVATION_THRESHOLD / mahalanobis_dist)
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
        self.P = self.P + np.eye(len(self.x)) * COVARIANCE_STABILIZER
        
        # Return innovation for history tracking
        return y
    
    # Modified process noise that accounts for accelerometer bias
    def update_process_noise(self, accel_measurements=None):
        """Update process noise based on measured accelerations."""
        sigma_a = ACCEL_UNCERTAINTY_BASE  # Base process noise
        
        if accel_measurements is not None:
            # Increase process noise when accelerations are detected
            accel_magnitude = np.sqrt(np.sum(np.square(accel_measurements)))
            
            # Scale up process noise based on acceleration magnitude
            # (larger accelerations mean more uncertainty in prediction)
            sigma_a = max(sigma_a, ACCEL_UNCERTAINTY_BASE + ACCEL_UNCERTAINTY_SCALE * accel_magnitude)
        
        # Rebuild Q matrix with updated sigma_a
        dt = self.F[0, 1]  # Extract dt from state transition matrix
        G = np.array([
            [dt**2/2, 0],
            [dt, 0],
            [0, dt**2/2],
            [0, dt]
        ])
        self.Q = G @ np.array([[sigma_a**2, 0], [0, sigma_a**2]]) @ G.T
    
    def check_filter_health(self):
        """Check if filter appears to be diverging and reset if necessary."""
        # Check if position variance is too large
        position_var = self.P[0, 0] + self.P[2, 2]
        
        # Check if innovation is consistently large
        if len(innovation_history) > FILTER_MINIMUM_INNOVATION_HISTORY:
            recent_innovations = np.array(list(innovation_history)[-FILTER_MINIMUM_INNOVATION_HISTORY:])
            avg_innovation = np.mean(np.abs(recent_innovations[:, :3]))  # Look at distance innovations
            
            # If both variance and innovation are large, filter may be diverging
            if position_var > FILTER_DIVERGENCE_POSITION_VAR_THRESHOLD and avg_innovation > FILTER_DIVERGENCE_INNOVATION_THRESHOLD:
                print("Warning: Filter may be diverging. Resetting covariance.")
                # Increase uncertainty but maintain position estimate
                self.P = np.diag([COVARIANCE_RESET_POSITION, COVARIANCE_RESET_VELOCITY, 
                                  COVARIANCE_RESET_POSITION, COVARIANCE_RESET_VELOCITY])
                return True
        
        return False


def validate_measurements(distance_measurements, accel_measurements=None, 
                         max_distance=MAX_VALID_DISTANCE, 
                         max_accel=MAX_VALID_ACCEL, 
                         min_distance=MIN_VALID_DISTANCE):
    """Validate measurements and reject outliers."""
    # Check distance measurements
    valid_distances = np.array(distance_measurements)
    
    # Check for maximum distances
    max_mask = valid_distances < max_distance
    
    # Check for minimum distances
    min_mask = valid_distances > min_distance
    
    # Combine masks
    mask = np.logical_and(max_mask, min_mask)
    
    if not np.all(mask):
        print(f"Warning: Rejected {np.sum(~mask)} invalid distance measurements")
        # Handle invalid measurements
        valid_distances = np.clip(valid_distances, min_distance, max_distance)
    
    # Check accelerometer measurements
    if accel_measurements is not None:
        valid_accel = np.array(accel_measurements)
        accel_mask = np.abs(valid_accel) < max_accel
        
        if not np.all(accel_mask):
            print(f"Warning: Rejected {np.sum(~accel_mask)} invalid acceleration measurements")
            # Cap accelerations to max value while preserving sign
            valid_accel = np.clip(valid_accel, -max_accel, max_accel)
        
        return valid_distances, valid_accel
    
    return valid_distances, None


def adaptive_measurement_covariance(distance_measurements, accel_measurements=None):
    """Dynamically adjust measurement covariance based on measurement quality."""
    # Initialize with base values
    R_distances = np.eye(len(distance_measurements)) * (STANDARD_DEVIATION ** 2)
    
    # Adjust based on measurement magnitude (larger distances typically have larger errors)
    for i, dist in enumerate(distance_measurements):
        # Scale the variance based on distance (simple linear scaling example)
        scale_factor = max(1.0, dist / DISTANCE_SCALING_THRESHOLD)
        R_distances[i, i] *= scale_factor
    
    if accel_measurements is not None:
        R_accel = np.eye(len(accel_measurements)) * (ACCEL_STANDARD_DEVIATION ** 2)
        
        # Increase uncertainty for larger acceleration values
        for i, accel in enumerate(accel_measurements):
            accel_scale = max(1.0, abs(accel) / ACCEL_SCALING_THRESHOLD)
            R_accel[i, i] *= accel_scale
            
        # Combine into full R matrix
        R_combined = np.zeros((len(distance_measurements) + len(accel_measurements), 
                              len(distance_measurements) + len(accel_measurements)))
        R_combined[:len(distance_measurements), :len(distance_measurements)] = R_distances
        R_combined[len(distance_measurements):, len(distance_measurements):] = R_accel
        
        return R_combined
    
    return R_distances


def find_robot_location(da, db, dc, ax=None, ay=None):
    """
    Main function to estimate robot location using EKF.
    
    Args:
        da, db, dc: Distances to beacons A, B, and C in centimeters
        ax, ay: Optional accelerometer readings in m/s²
    
    Returns:
        Estimated x, y position in centimeters
    """
    # Create measurement vectors
    distance_measurements = np.array([da, db, dc])
    accel_measurements = np.array([ax, ay]) if ax is not None and ay is not None else None
    
    # Create a simple low-pass filter for accelerometer data
    # Initialize filtered acceleration if not already done
    if not hasattr(find_robot_location, 'prev_accel'):
        find_robot_location.prev_accel = np.array([0.0, 0.0])

    if accel_measurements is not None:
        # Apply low-pass filter: y[n] = α*x[n] + (1-α)*y[n-1]
        filtered_accel = ACCEL_FILTER_ALPHA * accel_measurements + (1-ACCEL_FILTER_ALPHA) * find_robot_location.prev_accel
        find_robot_location.prev_accel = filtered_accel
        accel_measurements = filtered_accel
    
    # Update process noise based on accelerometer data
    ekf.update_process_noise(accel_measurements)
    
    # EKF predict step
    ekf.predict()
    
    # Calculate pre-update expected measurements for tracking purposes
    expected_z = ekf.compute_expected_measurements(accel_measurements is not None)
    
    # Perform update and get innovation
    innovation = ekf.update(distance_measurements, accel_measurements)
    
    # Store innovation in history
    innovation_history.append(innovation)
    
    # Check filter health and reset if necessary
    ekf.check_filter_health()
    
    # Store history for diagnostics
    ekf_estimates.append(np.array([ekf.x[0], ekf.x[2]]))
    P_history.append(np.array([ekf.P[0, 0], ekf.P[2, 2]]))
    
    # Return the estimated position
    return ekf.x[0], ekf.x[2]


# =============================================================================
# INITIALIZATION
# =============================================================================

# Set up fixed reference points
a = Point(BEACON_A_X, BEACON_A_Y)
b = Point(BEACON_B_X, BEACON_B_Y)
c = Point(BEACON_C_X, BEACON_C_Y)
fixed_points = [a, b, c]

# State transition matrix (x, vx, y, vy)
F = np.array([
    [1, TIME_STEP, 0, 0],          # x = x + vx*dt
    [0, 1, 0, 0],                  # vx = vx
    [0, 0, 1, TIME_STEP],          # y = y + vy*dt
    [0, 0, 0, 1]                   # vy = vy
])

# Process noise covariance matrix initialization
G = np.array([
    [TIME_STEP ** 2 / 2, 0],
    [TIME_STEP, 0],
    [0, TIME_STEP ** 2 / 2],
    [0, TIME_STEP]
])
initial_sigma_a = ACCEL_UNCERTAINTY_BASE
Q = G @ np.array([[initial_sigma_a ** 2, 0], [0, initial_sigma_a ** 2]]) @ G.T

# Initial state estimate - [x, vx, y, vy]
x0 = np.array([INIT_X, INIT_VX, INIT_Y, INIT_VY])

# Initial uncertainty
P0 = np.diag([INIT_POS_VARIANCE, INIT_VEL_VARIANCE, INIT_POS_VARIANCE, INIT_VEL_VARIANCE])

# Measurement noise covariance initialization
R_distances = np.eye(3) * (STANDARD_DEVIATION ** 2)
R_with_accel = np.zeros((5, 5))  # 3 distances + 2 accelerations
R_with_accel[:3, :3] = R_distances
R_with_accel[3:, 3:] = np.eye(2) * (ACCEL_STANDARD_DEVIATION ** 2)

# Create EKF instance
ekf = ExtendedKalmanFilter(F, Q, R_with_accel, x0, P0, fixed_points)

# Track the EKF estimates over time
ekf_estimates = deque(maxlen=MAX_HISTORY_SIZE)
innovation_history = deque(maxlen=MAX_HISTORY_SIZE)
P_history = deque(maxlen=MAX_HISTORY_SIZE)
