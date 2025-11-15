#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from point_processor import PointProcessor
from feature_detector import FeatureDetector
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('lunar_test')

class SimpleLogger:
    """Simple logger class that matches the interface expected by our processors"""
    def __init__(self):
        self.logger = logging.getLogger('lunar_test')
        
    def info(self, msg):
        self.logger.info(msg)
        
    def warn(self, msg):
        self.logger.warning(msg)
        
    def error(self, msg):
        self.logger.error(msg)

# Robot and LiDAR configuration
ROBOT_WIDTH = 68 # units: cm
ROBOT_LENGTH = 102  # units: cm
ROBOT_HEIGHT = 30.0  # cm

LIDAR_OFFSET_X = 0.0     # cm (0 = center of robot)
LIDAR_OFFSET_Y = 30.0    # cm (front of robot)
LIDAR_HEIGHT = 25.0      # cm from ground
LIDAR_PITCH = 45.0       # degrees (pointing downward at 45°)

# LiDAR specifications
LIDAR_RANGE_MIN = 20.0   # cm
LIDAR_RANGE_MAX = 400.0  # cm
LIDAR_FOV = 270.0        # degrees
LIDAR_ANGULAR_RESOLUTION = 1.0  # degrees

class RealisticLidarSimulator:
    def __init__(self, robot_dims=(ROBOT_WIDTH, ROBOT_LENGTH, ROBOT_HEIGHT),
                 lidar_pos=(LIDAR_OFFSET_X, LIDAR_OFFSET_Y, LIDAR_HEIGHT),
                 lidar_pitch=LIDAR_PITCH):
        """
        Initialize a realistic LiDAR simulator.
        
        Parameters:
        -----------
        robot_dims : tuple
            (width, length, height) of the robot in cm
        lidar_pos : tuple
            (offset_x, offset_y, height) of the LiDAR in cm
        lidar_pitch : float
            Pitch angle of the LiDAR in degrees (downward positive)
        """
        self.robot_width, self.robot_length, self.robot_height = robot_dims
        self.lidar_x, self.lidar_y, self.lidar_z = lidar_pos
        self.lidar_pitch = lidar_pitch
        
        # Convert pitch to radians
        self.lidar_pitch_rad = math.radians(lidar_pitch)
        
        # Calculate ground line where LiDAR beam intersects flat ground
        self._calculate_ground_line()
    
    def _calculate_ground_line(self):
        """Calculate the line on the ground where the LiDAR beam would intersect flat terrain."""
        # Distance from LiDAR to its projection on the ground
        self.projection_distance = self.lidar_z / math.tan(self.lidar_pitch_rad)
        
        # Distance from robot center to LiDAR ground projection (y-component)
        self.ground_y = self.lidar_y - self.projection_distance
        
        logger.info(f"LiDAR at ({self.lidar_x}, {self.lidar_y}, {self.lidar_z}) cm")
        logger.info(f"LiDAR beam projects to ground at y = {self.ground_y:.2f} cm")
    
    def generate_terrain(self, length=500.0, width=300.0, num_rocks=5, num_craters=3,
                         rock_size_range=(20, 40), crater_size_range=(30, 60),
                         noise_level=1.0, seed=None):
        """
        Generate synthetic lunar terrain with rocks (spheres) and craters (depressions).
        
        Parameters:
        -----------
        length : float
            Length of the terrain in cm (y-axis)
        width : float
            Width of the terrain in cm (x-axis)
        num_rocks : int
            Number of rocks to place
        num_craters : int
            Number of craters to place
        rock_size_range : tuple
            (min_radius, max_radius) for rocks in cm
        crater_size_range : tuple
            (min_radius, max_radius) for craters in cm
        noise_level : float
            Standard deviation of random noise added to terrain in cm
        seed : int or None
            Random seed for reproducibility
            
        Returns:
        --------
        terrain : dict
            Dictionary with terrain information including features
        """
        # Set random seed if provided
        if seed is not None:
            np.random.seed(seed)
        
        # Initialize terrain
        x_grid = np.linspace(-width/2, width/2, int(width))
        y_grid = np.linspace(-100, length-100, int(length))
        X, Y = np.meshgrid(x_grid, y_grid)
        Z = np.zeros_like(X) + np.random.normal(0, noise_level, X.shape)
        
        # Lists to store features
        rocks = []
        craters = []
        
        # Place rocks (spheres) on terrain
        for _ in range(num_rocks):
            # Random position
            x = np.random.uniform(-width/2 + 50, width/2 - 50)
            y = np.random.uniform(0, length - 100)
            
            # Random radius
            radius = np.random.uniform(rock_size_range[0], rock_size_range[1])
            
            # Add rock to list
            rocks.append((x, y, radius))
            
            # Apply rock to terrain (sphere)
            for i in range(Z.shape[0]):
                for j in range(Z.shape[1]):
                    dist_2d = np.sqrt((X[i, j] - x)**2 + (Y[i, j] - y)**2)
                    if dist_2d <= radius:
                        # Calculate height based on sphere equation
                        z_sphere = np.sqrt(radius**2 - dist_2d**2)
                        Z[i, j] = max(Z[i, j], z_sphere)
        
        # Place craters on terrain
        for _ in range(num_craters):
            # Random position
            x = np.random.uniform(-width/2 + 50, width/2 - 50)
            y = np.random.uniform(0, length - 100)
            
            # Random radius and depth
            radius = np.random.uniform(crater_size_range[0], crater_size_range[1])
            depth = radius * 0.3  # Depth proportional to radius
            
            # Add crater to list
            craters.append((x, y, radius))
            
            # Apply crater to terrain (inverted paraboloid)
            for i in range(Z.shape[0]):
                for j in range(Z.shape[1]):
                    dist_2d = np.sqrt((X[i, j] - x)**2 + (Y[i, j] - y)**2)
                    if dist_2d <= radius:
                        # Parabolic depression
                        z_crater = -depth * (1 - (dist_2d/radius)**2)
                        Z[i, j] += z_crater
        
        # Return terrain data
        return {
            'X': X,
            'Y': Y,
            'Z': Z,
            'rocks': rocks,
            'craters': craters
        }
    
    def simulate_lidar_scan(self, terrain, robot_pos=(0, 0, 0), robot_heading=0,
                           fov=LIDAR_FOV, angular_resolution=LIDAR_ANGULAR_RESOLUTION,
                           range_min=LIDAR_RANGE_MIN, range_max=LIDAR_RANGE_MAX,
                           visualize=True):
        """
        Simulate a LiDAR scan from the robot's position.
        
        Parameters:
        -----------
        terrain : dict
            Dictionary with terrain information including X, Y, Z grids
        robot_pos : tuple
            (x, y, z) position of the robot center in cm
        robot_heading : float
            Heading of the robot in degrees (0 = positive y-axis)
        fov : float
            Field of view of the LiDAR in degrees
        angular_resolution : float
            Angular resolution of the LiDAR in degrees
        range_min : float
            Minimum range of the LiDAR in cm
        range_max : float
            Maximum range of the LiDAR in cm
        visualize : bool
            Whether to create a visualization of the scan
            
        Returns:
        --------
        scan_points : numpy.ndarray
            Array of shape (N, 2) with (x, y) coordinates of scan points in robot frame
        """
        # Extract terrain data
        X = terrain['X']
        Y = terrain['Y']
        Z = terrain['Z']
        
        # Calculate LiDAR position in world coordinates
        robot_heading_rad = math.radians(robot_heading)
        
        # Rotation matrix
        cos_h = math.cos(robot_heading_rad)
        sin_h = math.sin(robot_heading_rad)
        
        # LiDAR position in world frame
        lidar_world_x = robot_pos[0] + (self.lidar_x * cos_h - self.lidar_y * sin_h)
        lidar_world_y = robot_pos[1] + (self.lidar_x * sin_h + self.lidar_y * cos_h)
        lidar_world_z = robot_pos[2] + self.lidar_z
        
        logger.info(f"Robot at ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}) with heading {robot_heading}°")
        logger.info(f"LiDAR in world frame: ({lidar_world_x:.2f}, {lidar_world_y:.2f}, {lidar_world_z:.2f})")
        
        # Calculate angles for LiDAR scan
        half_fov = fov / 2.0
        scan_angles = np.arange(-half_fov, half_fov + angular_resolution, angular_resolution)
        
        # Store scan points
        scan_points = []
        
        # For each angle, cast a ray
        for angle in scan_angles:
            # Combined angle (robot heading + LiDAR angle)
            world_angle = robot_heading + angle
            world_angle_rad = math.radians(world_angle)
            
            # Direction vector in x-y plane
            dx = math.sin(world_angle_rad)
            dy = math.cos(world_angle_rad)
            
            # Account for LiDAR pitch
            dz = -math.tan(self.lidar_pitch_rad)  # Negative because pointing downward
            
            # Normalize direction vector
            norm = math.sqrt(dx**2 + dy**2 + dz**2)
            dx /= norm
            dy /= norm
            dz /= norm
            
            # Cast ray and find intersection with terrain
            hit_point = self._cast_ray(
                X, Y, Z, 
                (lidar_world_x, lidar_world_y, lidar_world_z),
                (dx, dy, dz), 
                range_min, range_max
            )
            
            if hit_point is not None:
                # Transform point to robot frame
                hit_x, hit_y, hit_z = hit_point
                
                # Translate to robot's reference frame
                rel_x = hit_x - robot_pos[0]
                rel_y = hit_y - robot_pos[1]
                
                # Rotate to align with robot's heading
                rot_x = rel_x * cos_h + rel_y * sin_h
                rot_y = -rel_x * sin_h + rel_y * cos_h
                
                # Add to scan points (x, y) in robot frame
                scan_points.append((rot_x, rot_y))
        
        # Convert to numpy array
        scan_points = np.array(scan_points)
        
        # Visualize the scan if requested
        if visualize and len(scan_points) > 0:
            self._visualize_scan(terrain, robot_pos, robot_heading, scan_points)
        
        return scan_points
    
    def _cast_ray(self, X, Y, Z, origin, direction, min_range, max_range):
        """
        Cast a ray from origin in the given direction and find intersection with terrain.
        
        Parameters:
        -----------
        X, Y, Z : numpy.ndarray
            Terrain grids
        origin : tuple
            (x, y, z) origin of the ray
        direction : tuple
            (dx, dy, dz) normalized direction vector
        min_range : float
            Minimum range to consider
        max_range : float
            Maximum range to consider
            
        Returns:
        --------
        hit_point : tuple or None
            (x, y, z) coordinates of hit point, or None if no intersection
        """
        # Origin and direction
        ox, oy, oz = origin
        dx, dy, dz = direction
        
        # Step size for ray marching
        step_size = 2.0  # cm
        
        # Current distance along ray
        distance = min_range
        
        # Ray marching loop
        while distance < max_range:
            # Current point along ray
            px = ox + dx * distance
            py = oy + dy * distance
            pz = oz + dz * distance
            
            # Check if point is within terrain bounds
            if (px < np.min(X) or px > np.max(X) or
                py < np.min(Y) or py > np.max(Y)):
                # Out of bounds
                return None
            
            # Find closest terrain point
            i = np.argmin(np.abs(Y[:, 0] - py))
            j = np.argmin(np.abs(X[0, :] - px))
            
            # Get terrain height at this point
            terrain_z = Z[i, j]
            
            # Check if ray has intersected terrain
            if pz <= terrain_z:
                # Hit the terrain - refine hit point with binary search
                prev_distance = distance - step_size
                hit_distance = self._refine_hit_point(
                    X, Y, Z, origin, direction, prev_distance, distance
                )
                
                # Calculate hit point
                hit_x = ox + dx * hit_distance
                hit_y = oy + dy * hit_distance
                hit_z = oz + dz * hit_distance
                
                return (hit_x, hit_y, hit_z)
            
            # Increment distance
            distance += step_size
        
        # No intersection found within max range
        return None
    
    def _refine_hit_point(self, X, Y, Z, origin, direction, low_dist, high_dist, iterations=5):
        """
        Refine hit point using binary search.
        
        Parameters:
        -----------
        X, Y, Z : numpy.ndarray
            Terrain grids
        origin : tuple
            (x, y, z) origin of the ray
        direction : tuple
            (dx, dy, dz) normalized direction vector
        low_dist : float
            Lower bound of distance (known to be above terrain)
        high_dist : float
            Upper bound of distance (known to be below terrain)
        iterations : int
            Number of binary search iterations
            
        Returns:
        --------
        distance : float
            Refined distance to hit point
        """
        ox, oy, oz = origin
        dx, dy, dz = direction
        
        # Binary search
        for _ in range(iterations):
            mid_dist = (low_dist + high_dist) / 2.0
            
            # Calculate point
            px = ox + dx * mid_dist
            py = oy + dy * mid_dist
            pz = oz + dz * mid_dist
            
            # Find closest terrain point
            i = np.argmin(np.abs(Y[:, 0] - py))
            j = np.argmin(np.abs(X[0, :] - px))
            
            # Get terrain height
            terrain_z = Z[i, j]
            
            # Adjust bounds
            if pz <= terrain_z:
                # Below terrain
                high_dist = mid_dist
            else:
                # Above terrain
                low_dist = mid_dist
        
        # Return midpoint of final interval
        return (low_dist + high_dist) / 2.0
    
    def _visualize_scan(self, terrain, robot_pos, robot_heading, scan_points):
        """
        Create a visualization of the LiDAR scan.
        
        Parameters:
        -----------
        terrain : dict
            Dictionary with terrain information
        robot_pos : tuple
            (x, y, z) position of the robot
        robot_heading : float
            Heading of the robot in degrees
        scan_points : numpy.ndarray
            Array of shape (N, 2) with scan points in robot frame
        """
        # Create figure with 3D and 2D subplots
        fig = plt.figure(figsize=(14, 8))
        
        # 3D terrain plot
        ax1 = fig.add_subplot(121, projection='3d')
        
        # Plot terrain surface
        ax1.plot_surface(terrain['X'], terrain['Y'], terrain['Z'], 
                         cmap='terrain', alpha=0.7, edgecolor='none')
        
        # Convert scan points to world frame
        world_points = []
        robot_heading_rad = math.radians(robot_heading)
        cos_h = math.cos(robot_heading_rad)
        sin_h = math.sin(robot_heading_rad)
        
        for px, py in scan_points:
            # Rotate back to world frame
            wx = robot_pos[0] + px * cos_h - py * sin_h
            wy = robot_pos[1] + px * sin_h + py * cos_h
            
            # Find closest terrain point for z-value
            i = np.argmin(np.abs(terrain['Y'][:, 0] - wy))
            j = np.argmin(np.abs(terrain['X'][0, :] - wx))
            wz = terrain['Z'][i, j]
            
            world_points.append((wx, wy, wz))
        
        # Plot scan points
        if world_points:
            world_points = np.array(world_points)
            ax1.scatter(world_points[:, 0], world_points[:, 1], world_points[:, 2],
                       color='red', s=20, label='LiDAR Points')
        
        # Plot rocks and craters
        for x, y, radius in terrain['rocks']:
            # Simple sphere visualization
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 10)
            
            # Get terrain height at rock position
            i = np.argmin(np.abs(terrain['Y'][:, 0] - y))
            j = np.argmin(np.abs(terrain['X'][0, :] - x))
            terrain_z = terrain['Z'][i, j] - radius
            
            # Create sphere
            sx = x + radius * np.outer(np.cos(u), np.sin(v))
            sy = y + radius * np.outer(np.sin(u), np.sin(v))
            sz = terrain_z + radius * np.outer(np.ones_like(u), np.cos(v))
            
            # Plot wire frame
            ax1.plot_wireframe(sx, sy, sz, color='green', alpha=0.5)
        
        # Plot robot
        self._plot_robot(ax1, robot_pos, robot_heading)
        
        # LiDAR position in world frame
        lidar_world_x = robot_pos[0] + (self.lidar_x * cos_h - self.lidar_y * sin_h)
        lidar_world_y = robot_pos[1] + (self.lidar_x * sin_h + self.lidar_y * cos_h)
        lidar_world_z = robot_pos[2] + self.lidar_z
        
        # Plot LiDAR
        ax1.scatter([lidar_world_x], [lidar_world_y], [lidar_world_z], 
                   color='blue', s=50, marker='^', label='LiDAR')
        
        # Set labels and title
        ax1.set_xlabel('X (cm)')
        ax1.set_ylabel('Y (cm)')
        ax1.set_zlabel('Z (cm)')
        ax1.set_title('3D Terrain with LiDAR Scan')
        ax1.legend()
        
        # 2D plot (robot's perspective)
        ax2 = fig.add_subplot(122)
        
        # Plot scan points
        if len(scan_points) > 0:
            ax2.scatter(scan_points[:, 0], scan_points[:, 1], 
                      color='red', s=20, label='LiDAR Points')
        
        # Plot robot
        robot_rect = plt.Rectangle(
            (-self.robot_width/2, -self.robot_length/2 + self.lidar_y),
            self.robot_width, self.robot_length,
            angle=0, color='blue', alpha=0.5
        )
        ax2.add_patch(robot_rect)
        
        # Plot LiDAR position
        ax2.scatter([self.lidar_x], [0], color='blue', s=50, marker='^', label='LiDAR')
        
        # Set labels and title
        ax2.set_xlabel('X (cm)')
        ax2.set_ylabel('Y (cm)')
        ax2.set_title('2D LiDAR Scan (Robot Frame)')
        ax2.grid(True)
        ax2.axis('equal')
        
        # Set reasonable view limits based on LiDAR range
        ax2.set_xlim(-LIDAR_RANGE_MAX/2, LIDAR_RANGE_MAX/2)
        ax2.set_ylim(-LIDAR_RANGE_MAX/4, LIDAR_RANGE_MAX*3/4)
        
        ax2.legend()
        
        plt.tight_layout()
        plt.savefig('lidar_simulation.png')
    
    def _plot_robot(self, ax, position, heading):
        """Plot robot representation in 3D."""
        x, y, z = position
        heading_rad = math.radians(heading)
        
        # Calculate robot corners
        half_width = self.robot_width / 2
        half_length = self.robot_length / 2
        
        # Robot base coordinates
        corners_base = [
            [-half_width, -half_length, 0],
            [half_width, -half_length, 0],
            [half_width, half_length, 0],
            [-half_width, half_length, 0],
            [-half_width, -half_length, 0]
        ]
        
        # Robot top coordinates
        corners_top = [
            [-half_width, -half_length, self.robot_height],
            [half_width, -half_length, self.robot_height],
            [half_width, half_length, self.robot_height],
            [-half_width, half_length, self.robot_height],
            [-half_width, -half_length, self.robot_height]
        ]
        
        # Rotation matrix
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        
        # Transform corners to world coordinates
        world_corners_base = []
        world_corners_top = []
        
        for corner in corners_base:
            # Rotate
            rx = corner[0] * cos_h - corner[1] * sin_h
            ry = corner[0] * sin_h + corner[1] * cos_h
            
            # Translate
            wx = x + rx
            wy = y + ry
            wz = z + corner[2]
            
            world_corners_base.append([wx, wy, wz])
        
        for corner in corners_top:
            # Rotate
            rx = corner[0] * cos_h - corner[1] * sin_h
            ry = corner[0] * sin_h + corner[1] * cos_h
            
            # Translate
            wx = x + rx
            wy = y + ry
            wz = z + corner[2]
            
            world_corners_top.append([wx, wy, wz])
        
        # Convert to numpy arrays
        world_corners_base = np.array(world_corners_base)
        world_corners_top = np.array(world_corners_top)
        
        # Plot base and top
        ax.plot(world_corners_base[:, 0], world_corners_base[:, 1], world_corners_base[:, 2], 
               'b-', linewidth=2)
        ax.plot(world_corners_top[:, 0], world_corners_top[:, 1], world_corners_top[:, 2], 
               'b-', linewidth=2)
        
        # Plot vertical edges
        for i in range(4):
            ax.plot([world_corners_base[i, 0], world_corners_top[i, 0]],
                   [world_corners_base[i, 1], world_corners_top[i, 1]],
                   [world_corners_base[i, 2], world_corners_top[i, 2]],
                   'b-', linewidth=2)
        
        # Plot direction indicator
        front_center_base = [(world_corners_base[2, 0] + world_corners_base[3, 0]) / 2,
                             (world_corners_base[2, 1] + world_corners_base[3, 1]) / 2,
                             world_corners_base[2, 2]]
        front_center_top = [(world_corners_top[2, 0] + world_corners_top[3, 0]) / 2,
                           (world_corners_top[2, 1] + world_corners_top[3, 1]) / 2,
                           world_corners_top[2, 2]]
        
        ax.plot([front_center_base[0], front_center_top[0]],
               [front_center_base[1], front_center_top[1]],
               [front_center_base[2], front_center_top[2]],
               'r-', linewidth=3)

def simulate_and_process(lidar_simulator, terrain, robot_pos, robot_heading):
    """
    Simulate a LiDAR scan and process it with the feature detector.
    
    Parameters:
    -----------
    lidar_simulator : RealisticLidarSimulator
        Configured LiDAR simulator
    terrain : dict
        Terrain data
    robot_pos : tuple
        (x, y, z) robot position
    robot_heading : float
        Robot heading in degrees
        
    Returns:
    --------
    scan_points : numpy.ndarray
        LiDAR scan points
    processed_points : numpy.ndarray
        Processed scan points
    features : list
        Detected features
    """
    # Create the processors
    simple_logger = SimpleLogger()
    point_processor = PointProcessor(simple_logger)
    feature_detector = FeatureDetector(simple_logger)
    
    # 1. Simulate LiDAR scan
    scan_points = lidar_simulator.simulate_lidar_scan(
        terrain, robot_pos, robot_heading
    )
    
    # 2. Process the points
    processed_points = point_processor.preprocess(scan_points)
    
    # 3. Detect features
    features = feature_detector.detect_features(
        processed_points,
        height_threshold=5.0,
        min_distance=20.0,
        min_width=15.0,
        max_width=60.0
    )
    
    # 4. Visualize results
    plt.figure(figsize=(12, 8))
    
    # Plot original scan points
    plt.scatter(scan_points[:, 0], scan_points[:, 1], 
              s=5, color='gray', alpha=0.5, label='Raw Points')
    
    # Plot processed points
    plt.scatter(processed_points[:, 0], processed_points[:, 1], 
              s=5, color='blue', alpha=0.7, label='Processed Points')
    
    # Plot robot
    robot_rect = plt.Rectangle(
        (-ROBOT_WIDTH/2, -ROBOT_LENGTH/2 + LIDAR_OFFSET_Y),
        ROBOT_WIDTH, ROBOT_LENGTH,
        angle=0, color='blue', alpha=0.3
    )
    plt.gca().add_patch(robot_rect)
    
    # Plot detected features
    for i, (x, y, radius) in enumerate(features):
        circle = plt.Circle((x, y), radius, fill=False, edgecolor='red', linewidth=2)
        plt.gca().add_patch(circle)
        plt.text(x, y, f'F{i+1}', color='red', fontsize=10, ha='center', va='center')
    
    # Set labels and title
    plt.title('LiDAR Scan Processing Results')
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    
    # Reasonable limits
    plt.xlim(-LIDAR_RANGE_MAX/2, LIDAR_RANGE_MAX/2)
    plt.ylim(-LIDAR_RANGE_MAX/4, LIDAR_RANGE_MAX*3/4)
    
    plt.savefig('feature_detection_results.png')
    
    return scan_points, processed_points, features

def main():
    """Main function to demonstrate the realistic LiDAR simulator."""
    # Create the LiDAR simulator
    lidar_simulator = RealisticLidarSimulator()
    
    # Generate terrain
    print("Generating synthetic lunar terrain...")
    terrain = lidar_simulator.generate_terrain(
        length=600.0,
        width=400.0,
        num_rocks=7,
        num_craters=5,
        rock_size_range=(15, 40),
        crater_size_range=(20, 60),
        noise_level=1.0,
        seed=42
    )
    
    # Print terrain features
    print("\nTerrain features:")
    print("Rocks (x, y, radius):")
    for i, (x, y, radius) in enumerate(terrain['rocks']):
        print(f"{i+1}. Rock at ({x:.1f}, {y:.1f}) with radius {radius:.1f}cm")
    
    print("\nCraters (x, y, radius):")
    for i, (x, y, radius) in enumerate(terrain['craters']):
        print(f"{i+1}. Crater at ({x:.1f}, {y:.1f}) with radius {radius:.1f}cm")
    
    # Simulate LiDAR scan from robot position
    print("\nSimulating LiDAR scan...")
    robot_position = (0, 50, 0)  # (x, y, z) in cm
    robot_heading = 0.0  # degrees (facing positive y-direction)
    
    # Simulate and process data
    scan_points, processed_points, features = simulate_and_process(
        lidar_simulator, terrain, robot_position, robot_heading
    )
    
    # Print detected features
    print("\nDetected features:")
    for i, (x, y, radius) in enumerate(features):
        print(f"{i+1}. Feature at ({x:.1f}, {y:.1f}) with radius {radius:.1f}cm")
    
    # Create 3D visualization of terrain with scan
    print("\nCreating 3D terrain visualization...")
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot terrain surface
    ax.plot_surface(terrain['X'], terrain['Y'], terrain['Z'], 
                   cmap='terrain', alpha=0.7, edgecolor='none')
    
    # Set labels and title
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('3D Lunar Terrain')
    
    plt.savefig('lunar_terrain_3d.png')
    
    # Show plots if possible
    try:
        plt.show()
    except Exception as e:
        print(f"Could not display matplotlib plots: {e}. Check saved PNG files.")

if __name__ == "__main__":
    main()
