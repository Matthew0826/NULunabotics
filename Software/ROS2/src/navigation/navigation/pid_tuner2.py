import asyncio
import time
import numpy as np
from typing import Callable, Any, Tuple, List, Optional, Union
import matplotlib.pyplot as plt
from collections import deque
import logging
import threading

class PIDTuner:
    """
    A class for automatically tuning PID controllers for robotic systems using ROS2.
    
    This class provides functionality to tune PID controllers by running experiments
    and analyzing system response to optimize control parameters.
    """
    
    def __init__(self, 
                 settling_time_threshold: float = 0.02,
                 overshoot_threshold: float = 0.1,
                 steady_state_error_threshold: float = 0.01,
                 max_iterations: int = 20,
                 timeout: float = 30.0,
                 sampling_rate: float = 0.05,
                 logger=None):
        """
        Initialize the PID tuner with tuning parameters.
        
        Args:
            settling_time_threshold: Threshold for determining when the system has settled (as a fraction of setpoint)
            overshoot_threshold: Maximum allowed overshoot as a fraction of the step size
            steady_state_error_threshold: Acceptable steady state error as a fraction of the step size
            max_iterations: Maximum number of tuning iterations
            timeout: Maximum time in seconds for a single experiment
            sampling_rate: Time between measurements in seconds
            logger: Optional logger for recording tuning information
        """
        self.settling_time_threshold = settling_time_threshold
        self.overshoot_threshold = overshoot_threshold
        self.steady_state_error_threshold = steady_state_error_threshold
        self.max_iterations = max_iterations
        self.timeout = timeout
        self.sampling_rate = sampling_rate
        self.tuning_history = []
        self.logger = logger or logging.getLogger('pid_tuner')
        self.robot = None
        
    async def tune_pid(self, 
                       robot: Any,
                       pid_controller: Any,
                       feedback_function: Callable[[Any], float],
                       start_point: float,
                       end_point: float,
                       error_function: Callable[[float, float], float],
                       movement_function: Callable[[Any, float], None],
                       method: str = "ziegler_nichols",
                       plot_results: bool = True) -> Tuple[float, float, float]:
        """
        Tune a PID controller to achieve optimal response for a step input.
        
        Args:
            robot: The robot object controlling the system
            pid_controller: The PID controller to tune
            feedback_function: A function that returns the current state from the robot
            start_point: Initial state for tuning
            end_point: Target state for tuning (setpoint)
            error_function: Function to calculate error between current and target state
            movement_function: Async function to move the robot to a specific position/orientation
            method: Tuning method to use ("ziegler_nichols", "cohen_coon", or "manual")
            plot_results: Whether to plot the tuning results
            
        Returns:
            Tuple of optimized (Kp, Ki, Kd) gains
        """
        self.robot = robot
        self.log(f"Starting PID tuning from {start_point} to {end_point}")
        
        # Save original PID values to restore if needed
        original_Kp = pid_controller.Kp
        original_Ki = pid_controller.Ki
        original_Kd = pid_controller.Kd
        
        try:
            # Reset to start position
            await movement_function(start_point)
            await self.sleep(1.0)  # Allow system to stabilize
            
            if method == "ziegler_nichols":
                Kp, Ki, Kd = await self._ziegler_nichols_tuning(
                    robot, pid_controller, feedback_function, 
                    start_point, end_point, error_function, movement_function
                )
            elif method == "cohen_coon":
                Kp, Ki, Kd = await self._cohen_coon_tuning(
                    robot, pid_controller, feedback_function, 
                    start_point, end_point, error_function, movement_function
                )
            else:  # Default to manual tuning
                Kp, Ki, Kd = await self._manual_tuning(
                    robot, pid_controller, feedback_function, 
                    start_point, end_point, error_function, movement_function
                )
            
            # Apply the final tuned parameters
            pid_controller.Kp = Kp
            pid_controller.Ki = Ki
            pid_controller.Kd = Kd
            
            # Run a final test with the tuned parameters
            time_points, response = await self._run_step_test(
                robot, pid_controller, feedback_function,
                start_point, end_point, error_function, movement_function
            )
            
            # Analyze and print final performance
            metrics = self._analyze_step_response(time_points, response, start_point, end_point)
            self.log(f"Final PID parameters: Kp={Kp:.4f}, Ki={Ki:.4f}, Kd={Kd:.4f}")
            self.log(f"Performance metrics: ")
            self.log(f"  Rise time: {metrics['rise_time']:.2f}s")
            self.log(f"  Settling time: {metrics['settling_time']:.2f}s")
            self.log(f"  Overshoot: {metrics['overshoot']:.2f}%")
            self.log(f"  Steady-state error: {metrics['steady_state_error']:.4f}")
            
            if plot_results:
                self._plot_response(time_points, response, start_point, end_point)
                
            self.tuning_history.append({
                'Kp': Kp,
                'Ki': Ki,
                'Kd': Kd,
                'metrics': metrics,
                'time_points': time_points,
                'response': response
            })
            
            return Kp, Ki, Kd
            
        except Exception as e:
            self.log(f"Error during tuning: {e}", level="error")
            # Restore original values on error
            pid_controller.Kp = original_Kp
            pid_controller.Ki = original_Ki
            pid_controller.Kd = original_Kd
            raise
    
    async def _ziegler_nichols_tuning(self, 
                                     robot: Any, 
                                     pid_controller: Any,
                                     feedback_function: Callable[[Any], float],
                                     start_point: float,
                                     end_point: float,
                                     error_function: Callable[[float, float], float],
                                     movement_function: Callable[[Any, float], None]) -> Tuple[float, float, float]:
        """
        Implement Ziegler-Nichols tuning method.
        
        This method:
        1. Increases Kp until system oscillates (while Ki=Kd=0)
        2. Records the ultimate gain (Ku) and oscillation period (Tu)
        3. Sets PID parameters based on Ziegler-Nichols formulas
        """
        self.log("Starting Ziegler-Nichols tuning method")
        
        # Reset PID controller to P-only mode
        pid_controller.Ki = 0.0
        pid_controller.Kd = 0.0
        
        # Start with a conservative Kp value
        pid_controller.Kp = 0.1
        
        # Find the ultimate gain Ku by increasing Kp until sustained oscillations occur
        ku = None
        tu = None
        kp_increment = 0.1
        
        for iteration in range(self.max_iterations):
            # Run step test with current parameters
            await movement_function(start_point)
            await self.sleep(1.0)
            
            # Record the system response for analysis
            time_points, response = await self._record_step_response(
                robot, feedback_function, start_point, end_point, movement_function
            )
            
            # Check for sustained oscillations by analyzing the response
            oscillation_info = self._check_oscillations(time_points, response)
            
            if oscillation_info['has_sustained_oscillations']:
                ku = pid_controller.Kp
                tu = oscillation_info['period']
                self.log(f"Found ultimate gain Ku={ku:.4f} with oscillation period Tu={tu:.4f}s")
                break
            
            # If no oscillations yet, increase Kp
            old_kp = pid_controller.Kp
            pid_controller.Kp += kp_increment
            
            # If Kp becomes very large without oscillations, increase the increment
            if iteration > 5:
                kp_increment *= 1.5
                
            self.log(f"Iteration {iteration+1}: Kp={old_kp:.4f} → {pid_controller.Kp:.4f}")
        
        if ku is None or tu is None:
            self.log("Could not find ultimate gain within iteration limit. Using last tested value.")
            ku = pid_controller.Kp
            # Estimate Tu based on response characteristics
            oscillation_info = self._check_oscillations(time_points, response, force_estimate=True)
            tu = oscillation_info.get('period', 1.0)  # Default to 1.0 if estimation fails
            
        # Calculate PID parameters using Ziegler-Nichols formulas
        Kp = 0.6 * ku
        Ki = 1.2 * ku / tu
        Kd = 0.075 * ku * tu
        
        return Kp, Ki, Kd
    
    async def sleep(self, seconds: float):
        """Sleep for a specified number of seconds."""
        await self.robot.yield_once(seconds)
    
    async def _cohen_coon_tuning(self, 
                               robot: Any, 
                               pid_controller: Any,
                               feedback_function: Callable[[Any], float],
                               start_point: float,
                               end_point: float,
                               error_function: Callable[[float, float], float],
                               movement_function: Callable[[Any, float], None]) -> Tuple[float, float, float]:
        """
        Implement Cohen-Coon tuning method based on process reaction curve.
        """
        self.log("Starting Cohen-Coon tuning method")
        
        # Run open-loop step test (with minimal P control)
        pid_controller.Kp = 0.1
        pid_controller.Ki = 0.0
        pid_controller.Kd = 0.0
        
        await movement_function(start_point)
        await self.sleep(1.0)
        
        # Record the system response
        time_points, response = await self._record_step_response(
            robot, feedback_function, start_point, end_point, movement_function
        )
        
        # Process reaction curve analysis
        try:
            # Calculate process gain K
            delta_y = response[-1] - response[0]
            # make sure delta_y isn't 0
            if delta_y == 0:
                delta_y = 0.00001
            delta_u = end_point - start_point
            K = delta_y / delta_u
            
            # Find time delay (dead time) and time constant
            normalized_response = [(r - response[0]) / delta_y for r in response]
            t_delay_idx = next((i for i, r in enumerate(normalized_response) if r > 0.1), 1)
            t_63_idx = next((i for i, r in enumerate(normalized_response) if r > 0.632), len(normalized_response) - 1)
            
            t_delay = time_points[t_delay_idx]
            t_constant = time_points[t_63_idx] - t_delay
            if t_constant <= 0:
                t_constant = 0.01  # Avoid division by zero
            
            # Calculate Cohen-Coon parameters
            r = t_delay / t_constant
            
            Kp = (1/K) * (1.35 + 0.25 * r) / (1 + 0.6 * r)
            Ki = Kp / (t_constant * (0.54 + 0.33 * r))
            Kd = Kp * t_constant * (0.5 * r) / (1 + 0.3 * r)
            
            self.log(f"Cohen-Coon analysis: K={K:.4f}, t_delay={t_delay:.4f}s, t_constant={t_constant:.4f}s")
            
            return Kp, Ki, Kd
        
        except Exception as e:
            self.log(f"Error in Cohen-Coon analysis: {e}. Falling back to manual tuning.", level="warning")
            return await self._manual_tuning(
                robot, pid_controller, feedback_function, 
                start_point, end_point, error_function, movement_function
            )
    
    async def _manual_tuning(self, 
                            robot: Any, 
                            pid_controller: Any,
                            feedback_function: Callable[[Any], float],
                            start_point: float,
                            end_point: float,
                            error_function: Callable[[float, float], float],
                            movement_function: Callable[[Any, float], None]) -> Tuple[float, float, float]:
        """
        Implement iterative manual tuning by adjusting PID parameters based on performance metrics.
        """
        self.log("Starting manual iterative tuning")
        
        # Start with conservative PID values
        pid_controller.Kp = 0.5
        pid_controller.Ki = 0.0
        pid_controller.Kd = 0.1
        
        best_Kp = pid_controller.Kp
        best_Ki = pid_controller.Ki
        best_Kd = pid_controller.Kd
        best_score = float('inf')
        
        # Step size for parameter adjustments
        kp_step = 0.2
        ki_step = 0.05
        kd_step = 0.1
        
        for iteration in range(self.max_iterations):
            # Run step test with current parameters
            await movement_function(start_point)
            await self.sleep(1.0)
            
            # Record system response
            time_points, response = await self._record_step_response(
                robot, feedback_function, start_point, end_point, movement_function
            )
            
            # Analyze performance
            metrics = self._analyze_step_response(time_points, response, start_point, end_point)
            
            # Calculate score (lower is better)
            # Weights can be adjusted based on application priorities
            score = (
                2.0 * metrics['rise_time'] + 
                1.0 * metrics['settling_time'] + 
                5.0 * metrics['overshoot'] / 100.0 + 
                10.0 * abs(metrics['steady_state_error'])
            )
            
            self.log(f"Iteration {iteration+1}: Kp={pid_controller.Kp:.4f}, Ki={pid_controller.Ki:.4f}, Kd={pid_controller.Kd:.4f}")
            self.log(f"  Score: {score:.4f}, Rise: {metrics['rise_time']:.2f}s, Settling: {metrics['settling_time']:.2f}s, "
                  f"Overshoot: {metrics['overshoot']:.2f}%, Error: {metrics['steady_state_error']:.4f}")
            
            # Save best parameters
            if score < best_score:
                best_score = score
                best_Kp = pid_controller.Kp
                best_Ki = pid_controller.Ki
                best_Kd = pid_controller.Kd
                
            # Adjust parameters based on performance
            if metrics['rise_time'] > 1.0:
                # Too slow rise time - increase Kp
                pid_controller.Kp += kp_step
            elif metrics['overshoot'] > self.overshoot_threshold * 100:
                # Too much overshoot - decrease Kp, increase Kd
                pid_controller.Kp -= kp_step * 0.5
                pid_controller.Kd += kd_step
            elif abs(metrics['steady_state_error']) > self.steady_state_error_threshold:
                # Steady state error - increase Ki
                pid_controller.Ki += ki_step
            else:
                # Fine tuning
                # Try small adjustments in different directions
                if iteration % 3 == 0:
                    pid_controller.Kp *= 0.9
                elif iteration % 3 == 1:
                    pid_controller.Ki *= 1.1
                else:
                    pid_controller.Kd *= 0.9
            
            # Gradual reduction of step sizes for fine-tuning
            if iteration > self.max_iterations // 2:
                kp_step *= 0.8
                ki_step *= 0.8
                kd_step *= 0.8
                
            # Check for convergence
            if (score < 0.1 or 
                (iteration > 5 and abs(score - best_score) / best_score < 0.05)):
                self.log("Tuning converged")
                break
                
        # Return the best parameters found
        self.log(f"Best parameters found: Kp={best_Kp:.4f}, Ki={best_Ki:.4f}, Kd={best_Kd:.4f} with score {best_score:.4f}")
        return best_Kp, best_Ki, best_Kd
    async def _record_step_response(self,
                               robot: Any,
                               feedback_function: Callable[[Any], float],
                               start_point: float,
                               end_point: float,
                               movement_function: Callable[[Any, float], None]) -> Tuple[List[float], List[float]]:
        """
        Record the system response to a step input by monitoring feedback during movement.
        
        This method uses a ROS-compatible approach to monitoring the robot state.
        """
        time_points = []
        response = []
        start_time = time.time()
        
        # Record initial state
        initial_value = feedback_function(robot)
        time_points.append(0.0)
        response.append(initial_value)
        
        # Create a ROS timer for monitoring
        response_data = {"complete": False}
        
        # Define a callback for the timer
        def monitor_callback():
            if response_data["complete"]:
                return
                
            current_time = time.time() - start_time
            current_value = feedback_function(robot)
            
            time_points.append(current_time)
            response.append(current_value)
            
            # Stop monitoring after timeout
            if current_time > self.timeout:
                response_data["complete"] = True
        
        # Set up the timer using ROS Node's create_timer method if available
        if hasattr(robot, 'create_timer'):
            # ROS 2 node has create_timer method
            timer = robot.create_timer(self.sampling_rate, monitor_callback)
        else:
            # Fallback to a separate thread if not a ROS node
            stop_event = threading.Event()
            
            def threaded_monitor():
                while not stop_event.is_set():
                    monitor_callback()
                    time.sleep(self.sampling_rate)
                    
            monitor_thread = threading.Thread(target=threaded_monitor)
            monitor_thread.start()
        
        try:
            # Initiate the movement
            await movement_function(end_point)
            
            # Ensure we have some data after movement completes
            await self.sleep(2.0)
            
        finally:
            # Clean up the timer or thread
            response_data["complete"] = True
            
            if hasattr(robot, 'create_timer'):
                # Destroy the ROS timer
                robot.destroy_timer(timer)
            else:
                # Stop the monitoring thread
                stop_event.set()
                monitor_thread.join(timeout=1.0)
        
        return time_points, response

    async def _run_step_test(self, 
                            robot: Any, 
                            pid_controller: Any,
                            feedback_function: Callable[[Any], float],
                            start_point: float,
                            end_point: float,
                            error_function: Callable[[float, float], float],
                            movement_function: Callable[[Any, float], None]) -> Tuple[List[float], List[float]]:
        """
        Run a complete step response test with the current PID parameters.
        
        This is a wrapper around _record_step_response that ensures the system starts
        from the correct initial state.
        
        Returns:
            Tuple of (time_points, response_values)
        """
        # Ensure we're at the start point
        current = feedback_function(robot)
        if abs(error_function(current, start_point)) > 0.05 * abs(end_point - start_point):
            await movement_function(start_point)
            await self.sleep(1.0)
            
        # Record response to step input
        return await self._record_step_response(robot, feedback_function, start_point, end_point, movement_function)
    
    def _analyze_step_response(self, 
                              time_points: List[float], 
                              response: List[float],
                              start_point: float,
                              end_point: float) -> dict:
        """
        Analyze step response to calculate performance metrics.
        
        Returns:
            Dictionary with performance metrics: rise_time, settling_time, overshoot, steady_state_error
        """
        step_size = abs(end_point - start_point)
        if step_size == 0:
            return {'rise_time': 0, 'settling_time': 0, 'overshoot': 0, 'steady_state_error': 0}
            
        # Normalize response for analysis
        direction = 1 if end_point > start_point else -1
        normalized_response = [(r - start_point) / step_size * direction for r in response]
        
        # Calculate rise time (10% to 90%)
        try:
            idx_10 = next(i for i, r in enumerate(normalized_response) if r >= 0.1)
            idx_90 = next(i for i, r in enumerate(normalized_response) if r >= 0.9)
            rise_time = time_points[idx_90] - time_points[idx_10]
        except (StopIteration, IndexError):
            rise_time = time_points[-1] if time_points else 0  # Use full time if 90% is not reached
            
        # Calculate overshoot
        try:
            if direction > 0:
                max_value = max(normalized_response)
                overshoot = max(0, (max_value - 1.0) * 100)
            else:
                min_value = min(normalized_response)
                overshoot = max(0, (1.0 - min_value) * 100)
        except ValueError:
            overshoot = 0
                    
        # Calculate settling time (time to stay within ±2% of final value)
        settling_threshold = 0.02
        try:
            # Find the last time the response leaves the settling band
            settled_indices = [i for i, r in enumerate(normalized_response) 
                              if abs(r - 1.0) <= settling_threshold]
            
            if settled_indices:
                # Find the first index where the response stays within the band
                consecutive_count = 0
                settling_idx = len(normalized_response) - 1
                
                for i in range(len(normalized_response)):
                    if abs(normalized_response[i] - 1.0) <= settling_threshold:
                        consecutive_count += 1
                    else:
                        consecutive_count = 0
                        
                    # Consider settled if it stays in band for at least 5 samples
                    if consecutive_count >= 5:
                        settling_idx = i - 4  # The first index that started the sequence
                        break
                        
                settling_time = time_points[settling_idx]
            else:
                settling_time = time_points[-1] if time_points else 0
        except (IndexError, ValueError):
            settling_time = time_points[-1] if time_points else 0
            
        # Calculate steady state error
        if len(normalized_response) > 5:
            # Use average of last few points to reduce noise
            steady_state = sum(normalized_response[-min(10, len(normalized_response)):]) / min(10, len(normalized_response))
            steady_state_error = 1.0 - steady_state
        else:
            steady_state_error = 0
            
        return {
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot': overshoot,
            'steady_state_error': steady_state_error
        }
    
    def _check_oscillations(self, 
                           time_points: List[float], 
                           response: List[float],
                           force_estimate: bool = False) -> dict:
        """
        Check if the response has sustained oscillations and calculate oscillation period.
        
        Args:
            time_points: List of time points
            response: List of response values
            force_estimate: If True, estimate period even if oscillations aren't clear
            
        Returns:
            Dictionary with oscillation information
        """
        if len(response) < 10:
            return {'has_sustained_oscillations': False}
            
        # Compute differences to detect zero crossings
        response_mean = sum(response) / len(response)
        centered_response = [r - response_mean for r in response]
        
        # Detect zero crossings
        crossings = []
        for i in range(1, len(centered_response)):
            if centered_response[i-1] * centered_response[i] <= 0:
                # Linear interpolation for more accurate crossing time
                if centered_response[i-1] != centered_response[i]:
                    t_cross = time_points[i-1] + (time_points[i] - time_points[i-1]) * (
                        -centered_response[i-1] / (centered_response[i] - centered_response[i-1])
                    )
                else:
                    t_cross = time_points[i]
                    
                crossings.append(t_cross)
        
        # Need at least 4 crossings (2 periods) to confirm oscillations
        if len(crossings) < 4 and not force_estimate:
            return {'has_sustained_oscillations': False}
            
        # Calculate periods between zero crossings
        if len(crossings) >= 4:
            periods = []
            for i in range(0, len(crossings) - 2, 2):
                periods.append(crossings[i+2] - crossings[i])
                
            # Check if periods are consistent (sustained oscillations)
            if periods:
                avg_period = sum(periods) / len(periods)
                period_variation = max(abs(p - avg_period) for p in periods) / avg_period if avg_period else 1.0
                
                sustained = period_variation < 0.2 or force_estimate
                
                return {
                    'has_sustained_oscillations': sustained,
                    'period': avg_period,
                    'variation': period_variation
                }
        
        # If we don't have enough crossings but need an estimate
        if force_estimate:
            # Try to estimate period using FFT
            if len(response) > 20:
                n = len(response)
                sample_rate = n / (time_points[-1] - time_points[0]) if time_points[-1] > time_points[0] else 1.0
                fft = np.fft.fft(centered_response)
                freqs = np.fft.fftfreq(n, 1/sample_rate)
                
                # Get the frequency with maximum amplitude (excluding DC)
                idx = np.argmax(np.abs(fft[1:n//2])) + 1 if n > 2 else 1
                dominant_freq = abs(freqs[idx]) if idx < len(freqs) else 1.0
                
                if dominant_freq > 0:
                    return {
                        'has_sustained_oscillations': False,
                        'period': 1.0 / dominant_freq
                    }
            
            # Fallback - detect peaks
            try:
                from scipy import signal
                peaks, _ = signal.find_peaks(centered_response)
                if len(peaks) >= 2:
                    avg_peak_dist = sum(time_points[peaks[i+1]] - time_points[peaks[i]] 
                                      for i in range(len(peaks)-1)) / (len(peaks)-1)
                    return {
                        'has_sustained_oscillations': False,
                        'period': avg_peak_dist
                    }
            except:
                pass
        
        return {'has_sustained_oscillations': False}
    
    def _plot_response(self, time_points: List[float], response: List[float], start_point: float, end_point: float):
        """Plot the step response with performance metrics."""
        try:
            import matplotlib.pyplot as plt
            
            plt.figure(figsize=(10, 6))
            plt.plot(time_points, response, 'b-', linewidth=2)
            plt.axhline(y=start_point, color='r', linestyle='--', label='Start')
            plt.axhline(y=end_point, color='g', linestyle='--', label='Target')
            
            # Add settling range lines (±2%)
            settling_range = 0.02 * abs(end_point - start_point)
            plt.axhline(y=end_point + settling_range, color='g', linestyle=':', alpha=0.5)
            plt.axhline(y=end_point - settling_range, color='g', linestyle=':', alpha=0.5)
            
            # Add annotations for metrics
            metrics = self._analyze_step_response(time_points, response, start_point, end_point)
            
            plt.title('PID Controller Step Response')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Response')
            plt.grid(True)
            plt.legend()
            
            # Add text box with metrics
            text = f"Rise Time: {metrics['rise_time']:.2f}s\n" \
                   f"Settling Time: {metrics['settling_time']:.2f}s\n" \
                   f"Overshoot: {metrics['overshoot']:.2f}%\n" \
                   f"Steady State Error: {metrics['steady_state_error']:.4f}"
                   
            plt.annotate(text, xy=(0.02, 0.02), xycoords='axes fraction',
                        bbox=dict(boxstyle="round,pad=0.5", fc="lightyellow", alpha=0.8))
            
            plt.tight_layout()
            plt.savefig(f"pid_response_{time.strftime('%Y%m%d_%H%M%S')}.png")
            plt.show()
        except Exception as e:
            self.log(f"Error plotting response: {e}", level="warning")
    
    def get_tuning_history(self):
        """Return the history of tuning attempts for analysis."""
        return self.tuning_history
    
    def save_tuning_report(self, filename: str = "pid_tuning_report.txt"):
        """Save a tuning report to a file."""
        try:
            with open(filename, 'w') as f:
                f.write("PID Tuning Report\n")
                f.write("=================\n\n")
                
                for i, tuning in enumerate(self.tuning_history):
                    f.write(f"Tuning #{i+1}\n")
                    f.write(f"Parameters: Kp={tuning['Kp']:.4f}, Ki={tuning['Ki']:.4f}, Kd={tuning['Kd']:.4f}\n")
                    f.write(f"Performance Metrics:\n")
                    f.write(f"  Rise time: {tuning['metrics']['rise_time']:.2f}s\n")
                    f.write(f"  Settling time: {tuning['metrics']['settling_time']:.2f}s\n")
                    f.write(f"  Overshoot: {tuning['metrics']['overshoot']:.2f}%\n")
                    f.write(f"  Steady-state error: {tuning['metrics']['steady_state_error']:.4f}\n\n")
                    
            self.log(f"Tuning report saved to {filename}")
        except Exception as e:
            self.log(f"Error saving tuning report: {e}", level="error")
            
    def log(self, message, level="info"):
        """Log a message using the provided logger or print to console."""
        if self.logger:
            if level == "error":
                self.logger.error(message)
            elif level == "warning":
                self.logger.warning(message)
            else:
                self.logger.info(message)
        else:
            print(f"[PIDTuner] {message}")
