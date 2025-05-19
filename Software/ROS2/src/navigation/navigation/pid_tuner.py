import time
import asyncio
import numpy as np
from dataclasses import dataclass

@dataclass
class PIDPerformanceMetrics:
    rise_time: float
    overshoot: float
    settling_time: float
    steady_state_error: float
    oscillations: int

async def tune_pid(robot, pid, get_current_value, start_destination, end_destination, error_calculator, move_function, tolerance=1.0, max_time=10.0):
    """
    Automatically tune a PID controller for the robot.
    """
    # Parameter ranges centered around your working simulation values
    kp_values = np.linspace(0.005, 0.03, 6)  # Range around 0.015
    ki_values = np.linspace(0.0001, 0.001, 5)  # Range around 0.0005
    kd_values = np.linspace(0.0005, 0.004, 6)  # Range around 0.002
        
    best_params = {
        'Kp': 0.015,  # Initialize with working simulation values
        'Ki': 0.0005,
        'Kd': 0.002,
        'score': float('inf')  # Lower is better
    }
    
    async def test_pid_performance(kp, ki, kd):
        """Test PID parameters and return performance metrics"""
        # Set the PID parameters
        pid.Kp = kp
        pid.Ki = ki
        pid.Kd = kd
        
        robot.get_logger().info("PID reset start.")
        # Reset robot - don't wait for completion
        reset_task = asyncio.create_task(move_function(start_destination))
        await asyncio.sleep(0.25)  # Allow robot to stabilize
        
        # If reset is still running, cancel it - we need to start fresh
        if not reset_task.done():
            reset_task.cancel()
            try:
                await reset_task
            except asyncio.CancelledError:
                pass
        robot.get_logger().info("PID reset complete.")
        
        start_time = time.time()
        
        # Command the test without awaiting it
        # (we'll monitor progress separately)
        movement_task = asyncio.create_task(move_function(end_destination))
        
        # Track performance data
        history = []
        time_history = []
        reached_target = False
        rise_time = max_time
        max_overshoot = 0
        oscillation_count = 0
        last_direction = None
        
        # Monitor robot's response until max_time is reached
        while time.time() - start_time < max_time:
            current_time = time.time() - start_time
            current_value = get_current_value(robot)
            # Fix #1: Use error_calculator with correct argument order
            error = abs(error_calculator(end_destination, current_value))
            
            history.append(current_value)
            time_history.append(current_time)
            
            # Detect if we've reached the target for the first time
            if not reached_target and error <= tolerance:
                reached_target = True
                rise_time = current_time
            
            # Calculate overshoot - if we've gone past the target
            # For rotation, check if we've rotated too far
            # This needs special handling based on the nature of the error function
            if end_destination > start_destination:
                overshoot = max(0, current_value - end_destination)
            else:
                overshoot = max(0, end_destination - current_value)
            
            if overshoot > 0:
                max_overshoot = max(max_overshoot, overshoot)
            
            # Count oscillations
            if len(history) > 2:
                current_direction = history[-1] - history[-2]
                if last_direction is not None and (
                    (current_direction > 0 and last_direction < 0) or 
                    (current_direction < 0 and last_direction > 0)
                ):
                    oscillation_count += 1
                last_direction = current_direction
            
            await asyncio.sleep(0.05)  # Small sleep to avoid excessive sampling
        
        # Cancel the task if it's still running
        if not movement_task.done():
            movement_task.cancel()
            try:
                await movement_task
            except asyncio.CancelledError:
                pass
        
        # Get final error
        final_value = history[-1] if history else get_current_value(robot)
        # Fix #2: Use error_calculator with both arguments
        steady_state_error = abs(error_calculator(end_destination, final_value))
        
        # Calculate settling time (time to stay within tolerance band)
        settling_time = max_time
        if reached_target and len(history) > 1:
            for i in range(len(history) - 1, 0, -1):
                # Fix #3: Use error_calculator with correct order
                if abs(error_calculator(end_destination, history[i])) > tolerance:
                    settling_time = time_history[i+1] if i+1 < len(time_history) else max_time
                    break
        
        # Calculate overshoot percentage - fix #4
        # Get the total movement needed as denominator
        total_movement = abs(error_calculator(end_destination, start_destination))
        overshoot_percent = (max_overshoot / total_movement) * 100 if total_movement > 0 else 0
        
        return PIDPerformanceMetrics(
            rise_time=rise_time,
            overshoot=overshoot_percent,
            settling_time=settling_time,
            steady_state_error=steady_state_error,
            oscillations=oscillation_count // 2  # Convert direction changes to full oscillations
        )
    
    def calculate_score(metrics):
        """Calculate a score (lower is better) based on performance metrics"""
        # Weights for different performance aspects
        w_rise = 1.0
        w_overshoot = 2.0
        w_settling = 1.5
        w_error = 3.0
        w_oscillations = 2.0
        
        # Calculate score (lower is better)
        score = (
            w_rise * metrics.rise_time +
            w_overshoot * metrics.overshoot +
            w_settling * metrics.settling_time +
            w_error * (metrics.steady_state_error * 10) +  # Scale up error importance
            w_oscillations * metrics.oscillations
        )
        return score
    
    # Grid search through parameter combinations
    robot.get_logger().info("Starting PID tuning...")
    
    # First, find a good Kp (proportional gain)
    robot.get_logger().info("Tuning Kp...")
    for kp in kp_values:
        pid.Kp = kp
        pid.Ki = 0.0  # Start with only P control
        pid.Kd = 0.0
        
        metrics = await test_pid_performance(kp, 0.0, 0.0)
        score = calculate_score(metrics)
        robot.get_logger().info(f"Testing Kp={kp:.5f}: rise_time={metrics.rise_time:.2f}s, overshoot={metrics.overshoot:.2f}%, score={score:.2f}")
        
        if score < best_params['score']:
            best_params = {'Kp': kp, 'Ki': 0.0, 'Kd': 0.0, 'score': score}
    
    # Next, find a good Kd (derivative gain)
    robot.get_logger().info(f"Best Kp={best_params['Kp']:.5f}")
    robot.get_logger().info("Tuning Kd...")
    for kd in kd_values:
        metrics = await test_pid_performance(best_params['Kp'], 0.0, kd)
        score = calculate_score(metrics)
        robot.get_logger().info(f"Testing Kp={best_params['Kp']:.5f}, Kd={kd:.5f}: settling_time={metrics.settling_time:.2f}s, oscillations={metrics.oscillations}, score={score:.2f}")
        
        if score < best_params['score']:
            best_params = {'Kp': best_params['Kp'], 'Ki': 0.0, 'Kd': kd, 'score': score}
    
    # Finally, find a good Ki (integral gain)
    robot.get_logger().info(f"Best Kp={best_params['Kp']:.5f}, Kd={best_params['Kd']:.5f}")
    robot.get_logger().info("Tuning Ki...")
    for ki in ki_values:
        metrics = await test_pid_performance(best_params['Kp'], ki, best_params['Kd'])
        score = calculate_score(metrics)
        robot.get_logger().info(f"Testing Kp={best_params['Kp']:.5f}, Ki={ki:.5f}, Kd={best_params['Kd']:.5f}: steady_error={metrics.steady_state_error:.2f}, score={score:.2f}")
        
        if score < best_params['score']:
            best_params = {'Kp': best_params['Kp'], 'Ki': ki, 'Kd': best_params['Kd'], 'score': score}
    
    # Final refinement with smaller adjustments around best values
    robot.get_logger().info(f"Refining around best values: Kp={best_params['Kp']:.5f}, Ki={best_params['Ki']:.5f}, Kd={best_params['Kd']:.5f}")
    
    # Apply the best parameters
    pid.Kp = best_params['Kp']
    pid.Ki = best_params['Ki']
    pid.Kd = best_params['Kd']
    
    robot.get_logger().info(f"Tuning complete!")
    robot.get_logger().info(f"Best parameters: Kp={best_params['Kp']:.5f}, Ki={best_params['Ki']:.5f}, Kd={best_params['Kd']:.5f}")
    robot.get_logger().info(f"Final score: {best_params['score']:.2f}")
    
    return best_params['Kp'], best_params['Ki'], best_params['Kd']
