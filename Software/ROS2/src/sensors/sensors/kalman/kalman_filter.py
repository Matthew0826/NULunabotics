import numpy as np

from .data_filtering import validate_measurements

# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

# Measurement error parameters
MARGIN_OF_ERROR = 0.1  # Margin of error for distance measurements
STANDARD_DEVIATION = MARGIN_OF_ERROR / 1.96  # Standard deviation derived from margin of error
ACCEL_STANDARD_DEVIATION = 0.05  # Standard deviation for accelerometer measurements

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

# Map boundary parameters
BOUNDARY_BUFFER = 0.05  # Buffer zone near boundaries (as percentage of map dimension)

# Define the Extended Kalman Filter class with accelerometer and beacons as measurement
class ExtendedKalmanFilter:
    def __init__(self, x0, P0, map_width, map_height):
        # Default values - these are overridden when the robot position is queried
        self.F = np.eye(4)  # State transition matrix (4x4)
        self.Q = np.eye(4) * 0.01  # Process noise covariance (4x4)
        # Fixed points (beacons) for distance measurements
        # Unknown fixed points when ekf is initialized
        self.fixed_point_arrays = np.array([])
        # Number of fixed points (for measurement vector sizing)
        self.num_fixed_points = len(self.fixed_point_arrays)
        
        # Measurement noise covariance initialization
        R_distances = np.eye(3) * (STANDARD_DEVIATION ** 2)
        R_with_accel = np.zeros((5, 5))  # 3 distances + 2 accelerations
        R_with_accel[:3, :3] = R_distances
        R_with_accel[3:, 3:] = np.eye(2) * (ACCEL_STANDARD_DEVIATION ** 2)
        
        # Measurement noise covariance (includes both distance and accel measurements)
        self.R = R_with_accel
        self.x = x0  # Initial state estimate [x, vx, y, vy]
        self.P = P0  # Initial error covariance
        
        # Map boundaries
        self.map_width = map_width
        self.map_height = map_height
        self.buffer_x = map_width * BOUNDARY_BUFFER
        self.buffer_y = map_height * BOUNDARY_BUFFER

    def predict(self):
        """
        Predict the next state using the state transition model.
        This method updates the state estimate and covariance based on the process noise covariance.
        The predict function also applies boundary constraints to ensure the position remains within the map boundaries.
        """
        # Standard predict step (no control input)
        self.x = self.F @ self.x
        
        # Apply boundary constraints after prediction
        self.enforce_boundary_constraints()
        
        self.P = self.F @ self.P @ self.F.T + self.Q

    def enforce_boundary_constraints(self):
        """Ensure the position state remains within map boundaries."""
        # Position constraints (x coordinate)
        if self.x[0] < 0:
            # If outside left boundary
            self.x[0] = 0
            # Zero out horizontal velocity if moving leftward
            if self.x[1] < 0:
                self.x[1] = 0
        elif self.x[0] > self.map_width:
            # If outside right boundary
            self.x[0] = self.map_width
            # Zero out horizontal velocity if moving rightward
            if self.x[1] > 0:
                self.x[1] = 0
                
        # Position constraints (y coordinate)
        if self.x[2] < 0:
            # If outside top boundary
            self.x[2] = 0
            # Zero out vertical velocity if moving upward
            if self.x[3] < 0:
                self.x[3] = 0
        elif self.x[2] > self.map_height:
            # If outside bottom boundary
            self.x[2] = self.map_height
            # Zero out vertical velocity if moving downward
            if self.x[3] > 0:
                self.x[3] = 0
    
    def adjust_process_noise_near_boundary(self):
        """Adjust process noise when near boundaries to reflect increased uncertainty."""
        x, y = self.x[0], self.x[2]
        
        # Calculate distances to boundaries
        dist_to_left = x
        dist_to_right = self.map_width - x
        dist_to_top = y
        dist_to_bottom = self.map_height - y
        
        # Check if near any boundary (within buffer zone)
        near_x_boundary = (dist_to_left < self.buffer_x) or (dist_to_right < self.buffer_x)
        near_y_boundary = (dist_to_top < self.buffer_y) or (dist_to_bottom < self.buffer_y)
        
        if near_x_boundary or near_y_boundary:
            # Increase position uncertainty components when near boundaries
            # This accounts for potential reflections or constrained movement
            scaling_factor = 2.0  # Increased uncertainty near boundaries
            
            if near_x_boundary:
                self.Q[0, 0] *= scaling_factor  # Increase x position uncertainty
            
            if near_y_boundary:
                self.Q[2, 2] *= scaling_factor  # Increase y position uncertainty

    def compute_measurement_jacobian(self, with_accel=False):
        """
        Compute the Jacobian matrix (denoted by 'H') for the measurement model.
        A Jacobian matrix is used to linearize the measurement model around the current state estimate.
        The Jacobian is used in the Kalman filter update step to relate the state estimate to the measurements.
        Returns:
            H: Jacobian matrix of shape (num_measurements, state_dim)
        """
        # Calculate how many measurements we have (distances + accelerometer)
        num_measurements = self.num_fixed_points
        if with_accel:
            num_measurements += 2  # Add two more for ax and ay

        # Initialize H matrix with zeros
        H = np.zeros((num_measurements, len(self.x)))

        # Current state position
        x, y = self.x[0], self.x[2]

        # Calculate Jacobian entries for each fixed point (distance measurements)
        for i, point in enumerate(self.fixed_point_arrays):
            # Distance from current estimated position to fixed point
            dx = x - point[0]
            dy = y - point[1]
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
        """
        Compute expected measurements based on the current state estimate.
        Returns:
            expected_z: Expected measurements vector
        """
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
    
    def adaptive_measurement_covariance(self, distance_measurements, accel_measurements=None):
        """
        Dynamically adjust measurement covariance based on measurement quality.
        Returns:
            R: Adaptive measurement covariance matrix
        """
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
    
    def update(self, distance_measurements, accel_measurements=None):
        """
        Update the Kalman filter with new measurements.
        Uses Mahalanobis distance to check for large innovations.
        Innovations represent the difference between expected and actual measurements.
        Uses Joseph form for covariance update to ensure numerical stability.
        Args:
            distance_measurements: Distances to beacons A, B, and C in centimeters
            accel_measurements: Optional accelerometer readings in m/s²
        Returns:
            y: Innovation vector (difference between expected and actual measurements)
        """
        # Validate measurements first
        valid_distances, valid_accel = validate_measurements(distance_measurements, accel_measurements)
        
        # Determine if we have accelerometer measurements
        with_accel = valid_accel is not None
        
        # Adjust process noise if near boundary
        self.adjust_process_noise_near_boundary()
        
        # Calculate adaptive measurement covariance
        adaptive_R = self.adaptive_measurement_covariance(valid_distances, valid_accel)
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
        # Mahalanobis distance is used to check if the innovation is too large
        try:
            mahalanobis_dist = np.sqrt(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            # If inversion fails, continue without innovation check
            mahalanobis_dist = float('inf')
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
            
            # Implement a more sophisticated adaptive scaling method based on innovation magnitude
            if mahalanobis_dist > INNOVATION_THRESHOLD:
               # print(f"Large innovation detected (d={mahalanobis_dist:.2f}), applying adaptive scaling")
                
                # Nonlinear scaling that gradually reduces influence as innovation increases
                # Uses a sigmoid-like function to create a smooth transition
                # The scaling factor approaches 0 as mahalanobis_dist gets very large
                scaling_factor = INNOVATION_THRESHOLD / (mahalanobis_dist + 0.1 * (mahalanobis_dist - INNOVATION_THRESHOLD))
                
                # Apply scaling to Kalman gain
                K = K * scaling_factor
            elif mahalanobis_dist < INNOVATION_THRESHOLD * 0.5:
                # If innovation is particularly small (indicating high confidence measurements)
                # we can slightly increase the Kalman gain to give more weight to these measurements
                # This helps with quicker convergence when measurements are very reliable
                
                # Calculate boost factor (maximum 1.2x boost for very small innovations)
                boost_factor = 1.0 + 0.2 * (1.0 - mahalanobis_dist / (INNOVATION_THRESHOLD * 0.5))
                
                # Apply boosting to Kalman gain
                K = K * boost_factor
                
                if boost_factor > 1.05:  # Only log if boost is significant
                    print(f"Small innovation detected (d={mahalanobis_dist:.2f}), boosting influence by {boost_factor:.2f}x")
        except np.linalg.LinAlgError:
            # Fallback to pseudoinverse if regular inverse fails
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # Update state estimate
        self.x = self.x + K @ y
        
        # Apply boundary constraints after update
        self.enforce_boundary_constraints()
        
        # Joseph form for covariance update
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R @ K.T
        
        # Ensure symmetry and positive definiteness
        self.P = (self.P + self.P.T) / 2
        self.P = self.P + np.eye(len(self.x)) * COVARIANCE_STABILIZER
        
        # Return innovation for history tracking
        return y
    
    # Modified process noise that accounts for accelerometer bias
    def update_process_noise(self, dt: float, accel_measurements=None):
        """
        Update process noise based on measured accelerations.
        Tracks the accelerometer readings and adjusts the process noise covariance matrix accordingly.
        Args:
            dt: Time since last update in seconds
            accel_measurements: Optional accelerometer readings in m/s²
        """
        sigma_a = ACCEL_UNCERTAINTY_BASE  # Base process noise
        
        if accel_measurements is not None:
            # Increase process noise when accelerations are detected
            accel_magnitude = np.sqrt(np.sum(np.square(accel_measurements)))
            
            # Scale up process noise based on acceleration magnitude
            # (larger accelerations mean more uncertainty in prediction)
            sigma_a = max(sigma_a, ACCEL_UNCERTAINTY_BASE + ACCEL_UNCERTAINTY_SCALE * accel_magnitude)
        
        # Rebuild Q matrix with updated sigma_a
        G = np.array([
            [dt**2/2, 0],
            [dt, 0],
            [0, dt**2/2],
            [0, dt]
        ])
        self.Q = G @ np.array([[sigma_a**2, 0], [0, sigma_a**2]]) @ G.T
        
        # Adjust process noise if near boundary
        self.adjust_process_noise_near_boundary()
    
    def check_filter_health(self, innovation_history: list[np.ndarray]):
        """
        Check if filter appears to be diverging and reset if necessary.
        Args:
            innovation_history: History of innovations for diagnostics (type: list of 1D arrays of type np.float64)
        Returns:
            bool: True if filter was reset, False otherwise
        """
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
                
                # Make sure position is within boundaries after reset
                self.enforce_boundary_constraints()
                return True
        
        return False
