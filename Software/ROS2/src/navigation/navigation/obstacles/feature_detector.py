import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

class FeatureDetector:
    def __init__(self, logger):
        self.logger = logger
    
    def detect_features(self, points, height_threshold=10.0, min_distance=30.0, 
                        min_width=15.0, max_width=60.0):
        """
        Detect features (rocks and craters) using a simple height deviation approach.
        
        This detector simply looks for points that deviate significantly from the baseline
        (average y-value) and groups adjacent deviations into features.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing (x, y) coordinates of LiDAR points in cm
        height_threshold : float
            Minimum height/depth (in cm) to consider as a feature
        min_distance : float
            Minimum distance (in cm) between features
        min_width : float
            Minimum width (in cm) for a valid feature
        max_width : float
            Maximum width (in cm) for a valid feature
            
        Returns:
        --------
        features : list
            List of (x, y, radius) tuples containing detected features
        """
        # Check if we have enough points
        if len(points) < 10:
            self.logger.warn('Not enough points for feature detection')
            return []
            
        # Step 1: Sort points by x-coordinate
        sorted_indices = np.argsort(points[:, 0])
        sorted_points = points[sorted_indices]
        
        # Step 2: Calculate the baseline (median y value)
        baseline_y = np.median(sorted_points[:, 1])
        self.logger.info(f"Baseline y-value: {baseline_y:.2f} cm")
        
        # Step 3: Calculate deviations from baseline
        deviations = sorted_points[:, 1] - baseline_y
        
        # Step 4: Apply light smoothing to reduce noise
        smoothed_deviations = gaussian_filter1d(deviations, sigma=1.0)
        
        # Store for debugging
        self.sorted_points = sorted_points
        self.deviations = deviations
        self.smoothed_deviations = smoothed_deviations
        
        # Step 5: Detect positive peaks (rocks) and negative peaks (craters)
        # For rocks (positive peaks)
        rock_peaks, _ = find_peaks(smoothed_deviations, 
                                   height=height_threshold,
                                   distance=min_distance / (sorted_points[1, 0] - sorted_points[0, 0]))
        
        # For craters (negative peaks)
        crater_peaks, _ = find_peaks(-smoothed_deviations,
                                     height=height_threshold,
                                     distance=min_distance / (sorted_points[1, 0] - sorted_points[0, 0]))
        
        self.logger.info(f"Detected {len(rock_peaks)} potential rocks and {len(crater_peaks)} potential craters")
        
        # Step 6: Extract features from peaks
        features = []
        
        # Process rock features
        for peak_idx in rock_peaks:
            feature = self._extract_feature(sorted_points, peak_idx, smoothed_deviations, 
                                            feature_type="rock", height_threshold=height_threshold,
                                            min_width=min_width, max_width=max_width)
            if feature:
                features.append(feature)
        
        # Process crater features
        for peak_idx in crater_peaks:
            feature = self._extract_feature(sorted_points, peak_idx, smoothed_deviations, 
                                            feature_type="crater", height_threshold=height_threshold,
                                            min_width=min_width, max_width=max_width)
            if feature:
                features.append(feature)
        
        self.logger.info(f"Found {len(features)} valid features after filtering")
        return features
    
    def _extract_feature(self, sorted_points, peak_idx, deviations, feature_type, 
                         height_threshold, min_width, max_width):
        """
        Extract feature information from a detected peak.
        
        Parameters:
        -----------
        sorted_points : numpy.ndarray
            Array of points sorted by x-coordinate
        peak_idx : int
            Index of the peak in the sorted points array
        deviations : numpy.ndarray
            Array of deviations from baseline
        feature_type : str
            Type of feature ('rock' or 'crater')
        height_threshold : float
            Minimum height/depth to consider as a feature
        min_width : float
            Minimum width for a valid feature
        max_width : float
            Maximum width for a valid feature
            
        Returns:
        --------
        feature : tuple or None
            (x, y, radius) tuple if valid feature, None otherwise
        """
        # Get peak location
        peak_x = sorted_points[peak_idx, 0]
        peak_y = sorted_points[peak_idx, 1]
        
        # Value to check depends on feature type
        values_to_check = deviations if feature_type == "rock" else -deviations
        
        # Find left boundary (first point below threshold)
        left_idx = peak_idx
        while left_idx > 0 and values_to_check[left_idx] > height_threshold / 2:
            left_idx -= 1
        
        # Find right boundary (first point below threshold)
        right_idx = peak_idx
        while right_idx < len(sorted_points) - 1 and values_to_check[right_idx] > height_threshold / 2:
            right_idx += 1
        
        # Calculate feature width
        left_x = sorted_points[left_idx, 0]
        right_x = sorted_points[right_idx, 0]
        width = right_x - left_x
        
        # Check if width is valid
        if width < min_width or width > max_width:
            self.logger.info(f"Rejecting {feature_type} at x={peak_x:.2f}: width {width:.2f} cm outside range ({min_width:.1f}, {max_width:.1f})")
            return None
        
        # Calculate feature center and radius
        center_x = (left_x + right_x) / 2
        center_y = np.mean(sorted_points[left_idx:right_idx+1, 1])
        radius = width / 2
        
        # Log the detection
        height = abs(peak_y - np.median(sorted_points[:, 1]))
        self.logger.info(f"Detected {feature_type} at ({center_x:.2f}, {center_y:.2f}) with radius {radius:.2f} cm and height {height:.2f} cm")
        
        return (center_x, center_y, radius)
    
    def plot_detection_process(self, figsize=(12, 10)):
        """
        Create a visualization of the feature detection process.
        
        Parameters:
        -----------
        figsize : tuple
            Figure size (width, height) in inches
            
        Returns:
        --------
        fig : matplotlib.figure.Figure
            The figure object
        """
        if not hasattr(self, 'sorted_points') or not hasattr(self, 'smoothed_deviations'):
            raise ValueError("Detection process data not available. Run detect_features first.")
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=figsize)
        
        # Plot 1: Original points
        ax1.scatter(self.sorted_points[:, 0], self.sorted_points[:, 1], s=5, color='blue', alpha=0.7)
        ax1.axhline(y=np.median(self.sorted_points[:, 1]), color='k', linestyle='--', alpha=0.5, label='Baseline')
        ax1.set_title('LiDAR Points with Baseline')
        ax1.set_xlabel('X (cm)')
        ax1.set_ylabel('Y (cm)')
        ax1.grid(True, linestyle='--', alpha=0.7)
        ax1.legend()
        
        # Plot 2: Deviations with threshold
        ax2.plot(self.sorted_points[:, 0], self.deviations, 'b-', alpha=0.5, label='Raw Deviations')
        ax2.plot(self.sorted_points[:, 0], self.smoothed_deviations, 'g-', linewidth=2, label='Smoothed Deviations')
        
        # Add threshold lines
        height_threshold = 5.0  # Should match the value used in detect_features
        ax2.axhline(y=height_threshold, color='r', linestyle='--', alpha=0.7, label=f'Threshold (+{height_threshold})')
        ax2.axhline(y=-height_threshold, color='r', linestyle='--', alpha=0.7, label=f'Threshold (-{height_threshold})')
        
        # Shade regions
        ax2.fill_between(self.sorted_points[:, 0], height_threshold, 
                          np.max(self.smoothed_deviations)+1, color='green', alpha=0.2, label='Rock Region')
        ax2.fill_between(self.sorted_points[:, 0], np.min(self.smoothed_deviations)-1, 
                          -height_threshold, color='orange', alpha=0.2, label='Crater Region')
        
        ax2.set_title('Deviation Profile with Thresholds')
        ax2.set_xlabel('X (cm)')
        ax2.set_ylabel('Deviation from Baseline (cm)')
        ax2.grid(True, linestyle='--', alpha=0.7)
        ax2.legend()
        
        plt.tight_layout()
        return fig