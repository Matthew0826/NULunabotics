import numpy as np
from collections import deque

from sensors.kalman.data_filtering import LowPassFilter
from sensors.kalman.kalman_filter import ExtendedKalmanFilter

# Data history
MAX_HISTORY_SIZE = 100  # Maximum size of history deques

# EKF initialization
INIT_VX = 0.1  # Initial x velocity
INIT_VY = 0.1  # Initial y velocity
INIT_POS_VARIANCE = 0.5  # Initial position variance
INIT_VEL_VARIANCE = 0.05  # Initial velocity variance

# Low-pass filter parameters
ACCELERATION_FILTER_ALPHA = 0.3  # Low-pass filter coefficient (0 < alpha < 1)


class PositionTracker:
    """
    PositionTracker class for estimating the robot's position using an Extended Kalman Filter (EKF).
    
    This class uses distance measurements from beacons and optional accelerometer data to estimate
    the robot's position in a 2D space.
    """
    def __init__(self, beacon_positions, initial_position):
        # Initialize the low-pass filter for accelerometer data
        self.acceleration_filter = LowPassFilter(ACCELERATION_FILTER_ALPHA)

        # Initial state estimate and uncertainty - [x, vx, y, vy]
        x0 = np.array([initial_position[0], INIT_VX, initial_position[1], INIT_VY])
        P0 = np.diag([INIT_POS_VARIANCE, INIT_VEL_VARIANCE, INIT_POS_VARIANCE, INIT_VEL_VARIANCE])

        # Create EKF instance
        self.ekf = ExtendedKalmanFilter(x0, P0)
        self.position = initial_position
        
        # Set the fixed points (beacons)
        self.ekf.fixed_point_arrays = np.array([[p[0], p[1]] for p in beacon_positions])
        self.ekf.num_fixed_points = len(self.ekf.fixed_points)

        # Track the EKF estimates over time
        self.ekf_estimates = deque(maxlen=MAX_HISTORY_SIZE)
        self.innovation_history = deque(maxlen=MAX_HISTORY_SIZE)
        self.P_history = deque(maxlen=MAX_HISTORY_SIZE)

    def _update_ekf(self, delta_time, distance_measurements, accel_measurements=None):
        # Update state transition matrix for the new time step
        self.ekf.F = np.array([
            [1, delta_time, 0, 0],          # x = x + vx*dt
            [0, 1, 0, 0],                  # vx = vx
            [0, 0, 1, delta_time],          # y = y + vy*dt
            [0, 0, 0, 1]                   # vy = vy
        ])
        
        # Update process noise based on accelerometer data
        self.ekf.update_process_noise(delta_time, accel_measurements)
        
        # EKF predict step
        self.ekf.predict()
        
        # Calculate pre-update expected measurements for tracking purposes
        # expected_z = self.ekf.compute_expected_measurements(accel_measurements is not None)
        
        # Perform update and get innovation
        innovation = self.ekf.update(distance_measurements, accel_measurements)
        self.innovation_history.append(innovation)

    def update_tracker(self, delta_time, beacon_distances, acceleration):
        """
        The main function to estimate robot location using EKF.
        
        Args:
            da, db, dc: Distances to beacons A, B, and C in centimeters
            ax, ay: Optional accelerometer readings in m/s²
        
        Returns:
            Estimated x, y position in centimeters
        """
        
        # Create measurement vectors
        distance_measurements = np.array(beacon_distances)
        accel_measurements = self.acceleration_filter.filter(np.array(acceleration)) if acceleration is not None else None
        
        # Update the EKF with the new measurements
        self._update_ekf(delta_time, distance_measurements, accel_measurements)
        
        # Check filter health and reset if necessary
        self.ekf.check_filter_health(self.innovation_history)
        # Store history for diagnostics
        self.ekf_estimates.append(np.array([self.ekf.x[0], self.ekf.x[2]]))
        self.P_history.append(np.array([self.ekf.P[0, 0], self.ekf.P[2, 2]]))
        
        # Return the estimated position (x, y)
        self.position = (self.ekf.x[0], self.ekf.x[2])
