import numpy as np
from scipy import signal
from scipy.spatial import cKDTree

class PointProcessor:
    def __init__(self, logger):
        self.logger = logger
        
    def preprocess(self, points):
        """
        Preprocess 2D LiDAR points to handle noise removal, outlier detection,
        and apply Savitzky-Golay filtering.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing (x, y) coordinates of LiDAR points in cm
            
        Returns:
        --------
        filtered_points : numpy.ndarray
            Array of shape (M, 2) containing the filtered points
        """
        # Check if we have enough points to process
        if len(points) < 5:
            self.logger.warn('Not enough points for preprocessing')
            return points
        
        # Step 1: Convert to float32 for memory efficiency on Raspberry Pi
        points = points.astype(np.float32)
        
        # Step 2: Remove statistical outliers
        filtered_points = self._remove_statistical_outliers(points)
        
        # Step 3: Apply improved Savitzky-Golay filtering to smooth the points
        filtered_points = self._improved_savgol_filter(filtered_points)
        
        return filtered_points

    def _remove_statistical_outliers(self, points, k=8, std_ratio=2.0):
        """
        Remove outliers using the Statistical Outlier Removal method.
        
        This algorithm identifies points that are significantly distant from their
        neighboring points and removes them. It's especially useful for lunar terrain
        where dust/regolith can cause spurious readings.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing (x, y) coordinates
        k : int
            Number of nearest neighbors to consider
        std_ratio : float
            Standard deviation threshold for outlier identification
            
        Returns:
        --------
        filtered_points : numpy.ndarray
            Array of shape (M, 2) with outliers removed
        """
        # If fewer points than k+1, return the original points
        if len(points) <= k:
            return points
        
        # Memory-efficient implementation for Raspberry Pi
        # Calculate distances to k nearest neighbors for each point
        
        # Build KD tree for efficient nearest neighbor search
        tree = cKDTree(points)
        
        # For each point, find the k nearest neighbors
        distances, _ = tree.query(points, k=k+1)  # k+1 because point is its own neighbor
        
        # Calculate mean distance to k nearest neighbors (excluding itself)
        mean_distances = np.mean(distances[:, 1:], axis=1)
        
        # Calculate global mean and standard deviation of these distances
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        
        # Identify inliers (points with typical neighborhood distances)
        threshold = global_mean + std_ratio * global_std
        inlier_mask = mean_distances <= threshold
        
        # Return only the inlier points
        return points[inlier_mask]

    def _improved_savgol_filter(self, points, window_length=7, polyorder=2):
        """
        Apply an improved Savitzky-Golay filter to smooth the point data while preserving features.
        
        This improved version sorts points by their x-coordinate, which works better for horizontal
        lines. It also uses 'nearest' mode instead of 'wrap' to avoid edge artifacts.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing (x, y) coordinates
        window_length : int
            Length of the filter window (must be odd)
        polyorder : int
            Order of the polynomial used for fitting
            
        Returns:
        --------
        smoothed_points : numpy.ndarray
            Array of shape (N, 2) containing smoothed points
        """
        # Need enough points for the filter to work
        if len(points) < window_length:
            return points
        
        # For a horizontal line, sorting by x-coordinate makes more sense
        # Sort points by x-coordinate
        sorted_indices = np.argsort(points[:, 0])
        sorted_points = points[sorted_indices]
        
        # Apply Savitzky-Golay filter primarily to y coordinates
        # Using 'nearest' mode to avoid edge artifacts
        y_smoothed = signal.savgol_filter(
            sorted_points[:, 1],
            window_length,
            polyorder,
            mode='nearest'  # Changed from 'wrap' to 'nearest'
        ).astype(np.float32)
        
        # For x coordinates, we can either:
        # 1. Keep them as is (since we sorted by x, they should already be smooth)
        # 2. Apply a very light smoothing to reduce noise
        x_smoothed = signal.savgol_filter(
            sorted_points[:, 0],
            window_length,
            polyorder,
            mode='nearest',
            deriv=0  # No derivative
        ).astype(np.float32)
        
        # Combine smoothed coordinates
        smoothed_points = np.column_stack((x_smoothed, y_smoothed))
        
        # Restore original point order
        restore_indices = np.argsort(sorted_indices)
        return smoothed_points[restore_indices]

    def _apply_savgol_filter(self, points, window_length=7, polyorder=2):
        """
        ORIGINAL METHOD - Apply Savitzky-Golay filter to smooth the point data while preserving features.
        
        This filter is particularly effective for lunar terrain as it preserves the
        shape of features like crater edges while removing measurement noise.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing (x, y) coordinates
        window_length : int
            Length of the filter window (must be odd)
        polyorder : int
            Order of the polynomial used for fitting
            
        Returns:
        --------
        smoothed_points : numpy.ndarray
            Array of shape (N, 2) containing smoothed points
        """
        # Need enough points for the filter to work
        if len(points) < window_length:
            return points
        
        # Sort points by angle for proper filtering
        # Calculate angles of each point relative to origin
        angles = np.arctan2(points[:, 1], points[:, 0])
        sorted_indices = np.argsort(angles)
        sorted_points = points[sorted_indices]
        
        # Apply Savitzky-Golay filter to x and y coordinates separately
        # Use smaller data type (float32) for memory efficiency
        x_smoothed = signal.savgol_filter(
            sorted_points[:, 0],
            window_length,
            polyorder,
            mode='wrap'  # Wrap mode for circular scan data
        ).astype(np.float32)
        
        y_smoothed = signal.savgol_filter(
            sorted_points[:, 1],
            window_length,
            polyorder,
            mode='wrap'
        ).astype(np.float32)
        
        # Combine smoothed coordinates
        smoothed_points = np.column_stack((x_smoothed, y_smoothed))
        
        # Restore original point order
        restore_indices = np.argsort(sorted_indices)
        return smoothed_points[restore_indices]
