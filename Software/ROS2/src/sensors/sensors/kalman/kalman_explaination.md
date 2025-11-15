# Robot Location Tracking System

This code implements a location tracking system that uses distance measurements from fixed beacons and optional accelerometer data to figure out where a robot is. Let's break it down:

## What This Code Does

The system tracks a robot's position (x,y coordinates) by:
1. Taking distance measurements from three fixed beacons
2. Optionally using accelerometer data to improve accuracy
3. Applying statistical filtering to handle measurement errors
4. Estimating the most likely position based on all available data

## Core Components

### Main Classes

#### ExtendedKalmanFilter
This is the statistical heart of the system. It maintains an estimated state (position and velocity) and continuously refines it based on measurements.

```
Class ExtendedKalmanFilter:
    Initialize with:
        - Motion model (how we expect position to change over time)
        - Error models (how much we trust our predictions vs measurements)
        - Initial guess of position and velocity
        - Locations of fixed beacons
    
    Method predict():
        # Predict where robot should be now based on previous state
        Apply motion model to update state estimate
        Increase uncertainty to account for prediction errors
    
    Method update(distance_measurements, accel_measurements):
        # Validate incoming measurements first
        distances, accels = validate_measurements(...)
        
        # Calculate what measurements we'd expect to see if our current estimate is correct
        expected_distances = calculate_distances_from_current_estimate_to_beacons()
        
        # Compare actual to expected measurements (the difference is called "innovation")
        innovation = actual_measurements - expected_measurements
        
        # Calculate how much to trust these measurements
        adaptive_trust_factor = calculate_trust_based_on_measurement_quality()
        
        # Compute an optimal adjustment factor (balancing our prediction vs measurements)
        adjustment_factor = calculate_optimal_adjustment()
        
        # Update our position estimate using this balanced approach
        state = state + adjustment_factor * innovation
        
        # Update uncertainty estimates
        update_uncertainty_estimates()
        
        return innovation
        
    Method check_filter_health():
        # See if filter is working well or needs reset
        if position_uncertainty > THRESHOLD and consistent_large_errors:
            reset_uncertainty_to_reasonable_values()
            return True
        return False
```

### Helper Functions

```
Function validate_measurements(distance_measurements, accel_measurements):
    # Remove obviously wrong measurements
    Filter out distances < MIN_VALID_DISTANCE or > MAX_VALID_DISTANCE
    
    if accel_measurements provided:
        Filter out accelerations > MAX_VALID_ACCEL
        
    return cleaned_distances, cleaned_accels

Function adaptive_measurement_covariance(distances, accels):
    # Calculate how much to trust each measurement
    # Larger distances or accelerations get less trust
    
    for each distance:
        trust_factor = adjust_based_on_distance_magnitude()
    
    for each acceleration:
        trust_factor = adjust_based_on_acceleration_magnitude()
        
    return trust_factors
```

### Main Function

```
Function find_robot_location(distance_to_A, distance_to_B, distance_to_C, accel_x, accel_y):
    # Smooth accelerometer data with low-pass filter
    smoothed_accel = apply_low_pass_filter(accel_x, accel_y)
    
    # Update how much uncertainty to add in prediction step
    adjust_prediction_uncertainty(smoothed_accel)
    
    # Predict new position based on motion model
    ekf.predict()
    
    # Update estimate with new measurements
    innovation = ekf.update(distances, smoothed_accel)
    
    # Store history for diagnostics
    save_to_history(innovation, current_position, current_uncertainty)
    
    # Check if filter needs reset due to poor performance
    ekf.check_filter_health()
```

## How It Works (The Math Simplified)

1. **State Representation**: The system tracks four values in a vector: [x, vx, y, vy] where x,y are position and vx,vy are velocity

2. **Prediction Step**: At each time step (dt), the system predicts:
   ```
   new_x = x + vx*dt
   new_vx = vx
   new_y = y + vy*dt
   new_vy = vy
   ```

3. **Measurement Model**: It compares actual measurements with expected measurements:
   - Expected distance to each beacon = sqrt((x-beacon_x)² + (y-beacon_y)²)
   - Expected acceleration = 0 (assuming constant velocity)

4. **Update Step**: The system calculates:
   - Difference between actual and expected measurements
   - How much to trust each difference (based on distance size, acceleration size)
   - Optimal way to adjust current estimate based on new measurements
   - Updates position and velocity estimates accordingly

5. **Health Checks**: If uncertainty gets too high or measurements consistently disagree with predictions, the system resets its uncertainty estimates.

## Flow Summary

1. Initialize with a guess of robot position and reasonable uncertainty
2. For each new set of measurements:
   - Predict where robot should be now based on previous position and velocity
   - Compare beacon distances with expected distances
   - Calculate optimal adjustment to estimated position
   - Update position estimate and uncertainty values
   - Check if filter is working properly
   - Return best estimate of current position

This approach optimally balances the uncertainty in the motion model against the uncertainty in the sensor measurements, giving more weight to whichever is more reliable at any given moment.
