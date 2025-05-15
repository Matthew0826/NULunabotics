#!/usr/bin/env python3
import numpy as np
import logging
from point_processor import PointProcessor
from feature_detector import FeatureDetector
import matplotlib.pyplot as plt
import math

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

def generate_synthetic_points(num_points=100, line_y=0, noise_level=2.0, 
                              num_features=3, feature_height=15, feature_width=10):
    """
    Generate synthetic LiDAR-like 2D points that follow a horizontal line
    with occasional rock (convex) or crater (concave) features.
    
    Parameters:
    -----------
    num_points : int
        Number of points to generate
    line_y : float
        Y coordinate of the horizontal line
    noise_level : float
        Standard deviation of random noise added to points
    num_features : int
        Number of rocks/craters to add to the line
    feature_height : float
        Height/depth of features in cm
    feature_width : float
        Width of features in cm
        
    Returns:
    --------
    points : numpy.ndarray
        Array of shape (num_points, 2) with synthetic points
    feature_locations : list
        List of (x, y, radius, is_rock) for ground truth features
    """
    # Generate x coordinates evenly spaced over 400cm
    x = np.linspace(-100, 100, num_points)
    
    # Initialize y coordinates as a flat line
    y = np.ones_like(x) * line_y
    
    # Add noise to simulate LiDAR measurement errors
    y += np.random.normal(0, noise_level, num_points)
    
    # Create random features (rocks or craters)
    feature_locations = []
    
    for _ in range(num_features):
        # Randomly choose feature center
        feature_x = np.random.uniform(-80, 80)
        
        # Randomly decide if it's a rock (convex) or crater (concave)
        is_rock = np.random.choice([True, False])
        sign = 1 if is_rock else -1
        
        # Store feature for validation
        radius = (feature_width + np.random.uniform(-5, 5)) / 2
        feature_locations.append((feature_x, line_y, radius, is_rock))
        
        # Apply the feature to the points
        for i, x_val in enumerate(x):
            # Calculate distance from feature center
            distance = abs(x_val - feature_x)
            
            # If point is within feature width, adjust its height
            if distance < radius:
                # Use semi-circle equation to determine height at this point
                height_factor = math.sqrt(1 - (distance/radius)**2)
                y[i] += sign * feature_height * height_factor
    
    # Combine x and y coordinates
    points = np.column_stack((x, y))
    
    return points, feature_locations

def plot_points(points, title="Raw Points", figsize=(10, 6)):
    """
    Plot points using matplotlib.
    
    Parameters:
    -----------
    points : numpy.ndarray
        Array of shape (N, 2) containing (x, y) coordinates
    title : str
        Title for the plot
    figsize : tuple
        Figure size (width, height) in inches
    
    Returns:
    --------
    fig, ax : tuple
        Matplotlib figure and axes objects
    """
    fig, ax = plt.subplots(figsize=figsize)
    ax.scatter(points[:, 0], points[:, 1], s=5, color='blue', alpha=0.7)
    ax.set_title(title)
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.grid(True, linestyle='--', alpha=0.7)
    
    return fig, ax

def plot_features(points, features, ground_truth=None, title="Detected Features", figsize=(10, 6)):
    """
    Plot points and detected features using matplotlib.
    
    Parameters:
    -----------
    points : numpy.ndarray
        Array of shape (N, 2) containing (x, y) coordinates
    features : list
        List of (x, y, radius) tuples for detected features
    ground_truth : list or None
        Optional list of (x, y, radius, is_rock) tuples for ground truth features
    title : str
        Title for the plot
    figsize : tuple
        Figure size (width, height) in inches
    
    Returns:
    --------
    fig, ax : tuple
        Matplotlib figure and axes objects
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    # Plot points
    ax.scatter(points[:, 0], points[:, 1], s=5, color='blue', alpha=0.7, label='Points')
    
    # Plot detected features
    for i, (x, y, radius) in enumerate(features):
        circle = plt.Circle((x, y), radius, fill=False, edgecolor='red', linewidth=2)
        ax.add_patch(circle)
        ax.text(x, y, f'{i+1}', color='red', fontsize=10, ha='center', va='center')
    
    # Plot ground truth features if provided
    if ground_truth is not None:
        for i, (x, y, radius, is_rock) in enumerate(ground_truth):
            color = 'orange' if is_rock else 'purple'
            label = 'R' if is_rock else 'C'
            circle = plt.Circle((x, y), radius, fill=False, edgecolor=color, 
                               linewidth=2, linestyle='--')
            ax.add_patch(circle)
            ax.text(x-radius/2, y-radius/2, f'{i+1}{label}', color=color, 
                   fontsize=10, ha='center', va='center')
    
    ax.set_title(title)
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # Create legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=8, label='Points'),
        Line2D([0], [0], color='red', lw=2, label='Detected Features'),
    ]
    
    if ground_truth is not None:
        legend_elements.extend([
            Line2D([0], [0], color='orange', lw=2, linestyle='--', label='Ground Truth Rock'),
            Line2D([0], [0], color='purple', lw=2, linestyle='--', label='Ground Truth Crater')
        ])
    
    ax.legend(handles=legend_elements, loc='best')
    
    return fig, ax

def main():
    # Create our processors
    simple_logger = SimpleLogger()
    point_processor = PointProcessor(simple_logger)
    feature_detector = FeatureDetector(simple_logger)
    
    # Generate synthetic LiDAR data with features
    print("Generating synthetic LiDAR data...")
    # points, ground_truth = generate_synthetic_points(
    #     num_points=200,
    #     line_y=-50,  # Base line at y=50cm
    #     noise_level=2.0,
    #     num_features=3,
    #     feature_height=10,
    #     feature_width=35
    # )
    points = []
    # read points from file
    with open("/home/isaac/NULunabotics/Software/ROS2/src/navigation/navigation/obstacles/data.txt", "r") as f:
        for line in f:
            x, y = map(float, line.strip().split(","))
            if abs(x) < 300:
                points.append((x, y))
    points = np.array(points)
    
    
    # Print ground truth features
    # print("\nGround truth features (x, y, radius, is_rock):")
    # for i, (x, y, radius, is_rock) in enumerate(ground_truth):
    #     feature_type = "Rock" if is_rock else "Crater"
    #     print(f"{i+1}. {feature_type} at ({x:.1f}, {y:.1f}) with radius {radius:.1f}cm")
    
    # Create a figure with multiple subplots
    plt.figure(figsize=(12, 10))
    
    # Plot 1: Raw points
    plt.subplot(3, 1, 1)
    plt.scatter(points[:, 0], points[:, 1], s=5, color='blue', alpha=0.7)
    plt.title('Raw LiDAR Points')
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Step 1: Preprocess the points
    print("Preprocessing points...")
    filtered_points = point_processor.preprocess(points)
    
    # Plot 2: Processed points
    plt.subplot(3, 1, 2)
    plt.scatter(filtered_points[:, 0], filtered_points[:, 1], s=5, color='green', alpha=0.7)
    plt.title('Processed Points')
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Step 2: Detect features
    print("Detecting features...")
    features = feature_detector.detect_features(filtered_points)
    
    print(f"\nDetected {len(features)} features:")
    for i, (x, y, radius) in enumerate(features):
        print(f"{i+1}. Feature at ({x:.1f}, {y:.1f}) with radius {radius:.1f}cm")
    
    # Plot 3: Detected features
    plt.subplot(3, 1, 3)
    plt.scatter(filtered_points[:, 0], filtered_points[:, 1], s=5, color='green', alpha=0.7, label='Processed Points')
    
    # Plot detected features
    for i, (x, y, radius) in enumerate(features):
        circle = plt.Circle((x, y), radius, fill=False, edgecolor='red', linewidth=2)
        plt.gca().add_patch(circle)
        plt.gca().text(x, y, f'{i+1}', color='red', fontsize=10, ha='center', va='center')
    
    # # Plot ground truth features
    # for i, (x, y, radius, is_rock) in enumerate(ground_truth):
    #     color = 'orange' if is_rock else 'purple'
    #     label = 'R' if is_rock else 'C'
    #     circle = plt.Circle((x, y), radius, fill=False, edgecolor=color, 
    #                        linewidth=2, linestyle='--')
    #     plt.gca().add_patch(circle)
    #     plt.gca().text(x-radius/2, y-radius/2, f'{i+1}{label}', color=color, 
    #                    fontsize=10, ha='center', va='center')
    
    plt.title('Detected Features vs Ground Truth')
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.grid(True, linestyle='--', alpha=0.7)
    
    # Create legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=8, label='Processed Points'),
        Line2D([0], [0], color='red', lw=2, label='Detected Features'),
        Line2D([0], [0], color='orange', lw=2, linestyle='--', label='Ground Truth Rock'),
        Line2D([0], [0], color='purple', lw=2, linestyle='--', label='Ground Truth Crater')
    ]
    plt.legend(handles=legend_elements, loc='best')
    
    try:
        plt.show()
    except Exception as e:
        print(f"Could not display matplotlib plot: {e}. Check the saved PNG files.")

if __name__ == "__main__":
    main()