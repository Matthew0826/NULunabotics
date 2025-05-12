import numpy as np
from scipy.ndimage import gaussian_filter1d
from itertools import groupby
from operator import itemgetter

# in cm
MIN_CRATER_SIZE = 40
MAX_CRATER_SIZE = 50
MIN_ROCK_SIZE = 30
MAX_ROCK_SIZE = 40

class FeatureDetector:
    def __init__(self, logger):
        self.logger = logger
    
    def detect_features(self, points, crater_size_range=(MIN_CRATER_SIZE, MAX_CRATER_SIZE), rock_size_range=(MIN_ROCK_SIZE, MAX_ROCK_SIZE)):
        """
        Detect craters (concave) and rocks (convex) in preprocessed LiDAR points
        using derivative-based and curvature analysis.
        
        This function:
        1. Calculates first/second derivatives to find edges
        2. Analyzes curvature sign to differentiate concave and convex features
        3. Classifies features as craters or rocks based on size and curvature
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing preprocessed (x, y) coordinates
        crater_size_range : tuple
            (min_size, max_size) for crater detection in cm
        rock_size_range : tuple
            (min_size, max_size) for rock detection in cm
            
        Returns:
        --------
        features : dict
            Dictionary with 'craters' and 'rocks' keys, each containing a list of
            detected features with position and size information
        """
        # Check if we have enough points to process
        if len(points) < 10:
            self.logger.warn('Not enough points for feature detection')
            return []
        
        # Step 1: Sort points by angle for proper curve representation
        # This is essential for derivative calculations
        angles = np.arctan2(points[:, 1], points[:, 0])
        sorted_indices = np.argsort(angles)
        sorted_points = points[sorted_indices]
        
        # Step 2: Calculate signed curvature
        curvature = self._compute_signed_curvature(sorted_points)
        
        # Step 3: Find regions of significant curvature
        # These are potential feature locations
        features = self._find_features_by_curvature(sorted_points, curvature)
        
        # Step 4: Filter features by size and classify
        # craters, rocks = self._classify_features_by_size(
        #     features, 
        #     crater_size_range,
        #     rock_size_range
        # )
        
        return features

    def _compute_signed_curvature(self, points):
        """
        Compute signed curvature for a sequence of 2D points.
        
        Positive curvature indicates convex regions (rocks)
        Negative curvature indicates concave regions (craters)
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing sorted (x, y) coordinates in cm
            
        Returns:
        --------
        curvature : numpy.ndarray
            Array of shape (N,) containing signed curvature values
        """
        # We need at least 3 points to calculate curvature
        if len(points) < 3:
            return np.zeros(len(points))
        
        # Compute first derivatives (dx/dt, dy/dt) using central differences
        # The 't' parameter is just the index of the point in the sequence
        dx = np.gradient(points[:, 0])
        dy = np.gradient(points[:, 1])
        
        # Compute second derivatives
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # Calculate curvature: k = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
        numerator = dx * ddy - dy * ddx
        denominator = (dx**2 + dy**2)**(1.5)
        
        # Handle division by zero
        curvature = np.zeros_like(numerator)
        mask = denominator > 1e-10
        curvature[mask] = numerator[mask] / denominator[mask]
        
        # Apply gaussian smoothing to reduce noise in curvature values
        curvature = gaussian_filter1d(curvature, sigma=1.0)
        
        return curvature

    def _find_features_by_curvature(self, points, curvature, threshold=0.5):
        """
        Identify feature regions based on curvature values.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Array of shape (N, 2) containing sorted (x, y) coordinates
        curvature : numpy.ndarray
            Array of shape (N,) containing signed curvature values
        threshold : float
            Curvature threshold for feature detection
            
        Returns:
        --------
        features : list
            List of dict objects containing feature information
        """
        features = []
        
        # Find regions with significant positive or negative curvature
        # First, identify points with high curvature magnitude
        high_curvature = np.abs(curvature) > threshold
        
        # Convert boolean array to indices
        high_curvature_indices = np.where(high_curvature)[0]
        
        # If no high curvature points, return empty list
        if len(high_curvature_indices) == 0:
            return features
        
        # Group adjacent indices to identify continuous regions
        for k, g in groupby(enumerate(high_curvature_indices), lambda x: x[0] - x[1]):
            region_indices = list(map(itemgetter(1), g))
            
            # Need at least 3 points to define a feature
            if len(region_indices) < 3:
                continue
            
            # Extract points in this region
            region_points = points[region_indices]
            
            # Calculate average curvature for the region
            avg_curvature = np.mean(curvature[region_indices])
            
            # Calculate region size (maximum distance across the region)
            min_coords = np.min(region_points, axis=0)
            max_coords = np.max(region_points, axis=0)
            size = np.max(max_coords - min_coords)
            
            # Calculate region center
            center = np.mean(region_points, axis=0)
            
            x_cm = center[0]
            y_cm = center[1]
            radius_cm = size / 2
            self.logger.info(f"Feature detected at ({x_cm:.2f}, {y_cm:.2f}) with radius {radius_cm:.2f} cm and curvature {avg_curvature:.2f}")
            # Add feature to list
            features.append((
                x_cm, y_cm, radius_cm
            ))
        
        return features

    def _classify_features_by_size(self, features, crater_size_range, rock_size_range):
        """
        Classify features as craters or rocks based on size and curvature sign.
        
        Parameters:
        -----------
        features : list
            List of dict objects containing feature information
        crater_size_range : tuple
            (min_size, max_size) for crater detection in meters
        rock_size_range : tuple
            (min_size, max_size) for rock detection in meters
            
        Returns:
        --------
        craters : list
            List of dict objects containing crater information
        rocks : list
            List of dict objects containing rock information
        """
        craters = []
        rocks = []
        
        for feature in features:
            size = feature['size']
            curvature = feature['curvature']
            
            # Negative curvature = concave = crater
            if curvature < 0 and crater_size_range[0] <= size <= crater_size_range[1]:
                craters.append({
                    'center': feature['center'],
                    'size': size
                })
            
            # Positive curvature = convex = rock
            elif curvature > 0 and rock_size_range[0] <= size <= rock_size_range[1]:
                rocks.append({
                    'center': feature['center'],
                    'size': size
                })
        
        return craters, rocks
