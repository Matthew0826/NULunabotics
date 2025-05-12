import numpy as np
import math


# Validation thresholds
MIN_VALID_DISTANCE = 1.0  # Minimum valid distance measurement (cm)
MAX_VALID_DISTANCE = 15000  # Maximum valid distance measurement (cm)
MAX_VALID_ACCEL = 10.0  # Maximum valid acceleration (m/s²)


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
        print(f"Warning: Rejected {np.sum(~mask)} invalid distance measurements ({distance_measurements})")
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

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev_value = None

    def filter(self, value):
        if self.prev_value is None:
            self.prev_value = value
        else:
            # Apply low-pass filter: y[n] = α*x[n] + (1-α)*y[n-1]
            value = self.alpha * value + (1 - self.alpha) * self.prev_value
        self.prev_value = value
        return value
 