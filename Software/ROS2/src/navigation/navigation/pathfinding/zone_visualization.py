import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import math

from zone import Zone

# Constants from the provided files
MAP_WIDTH = 550  # cm
MAP_HEIGHT = 490  # cm
GRID_RESOLUTION = 2  # cm
BERM_DUMP_SIZE = 10  # cm


# enum for zones on the map
OUT_OF_BOUNDS = -1
START_ZONE = 0
TRAVERSAL_ZONE = 1
EXCAVATION_ZONE = 2
DUMP_ZONE = 3
BERM_ZONE = 4


# Create figure and axis
plt.figure(figsize=(10, 9))
ax = plt.gca()

# Function to get zone type (copied from pathfinder_helper.py)
def get_zone(x, y):
    """Returns the zone of the map that the given coordinates are in (world coordinates).
    Options are START_ZONE, TRAVERSAL_ZONE, EXCAVATION_ZONE, DUMP_ZONE, BERM_ZONE, OUT_OF_BOUNDS."""
    y = MAP_HEIGHT - y  # Flip y-axis for visualization
    if x < 0 or x >= MAP_WIDTH or y < 0 or y >= MAP_HEIGHT:
        return OUT_OF_BOUNDS
    if y < 200:
        if x > (MAP_WIDTH - 200):
            return START_ZONE
        return TRAVERSAL_ZONE
    if y < (MAP_HEIGHT - 243):
        return TRAVERSAL_ZONE
    if x < 274:
        return EXCAVATION_ZONE
    if (x > (MAP_WIDTH - (70 + 50))) and (x < MAP_WIDTH - 50) and (y > (MAP_HEIGHT - 200 - 43/2)) and (y < MAP_HEIGHT - 43/2):
        return BERM_ZONE
    return DUMP_ZONE

# Create a colormap for the zones
colors = {
    -1: 'white',         # OUT_OF_BOUNDS
    0: 'lightblue',      # START_ZONE
    1: 'lightgray',      # TRAVERSAL_ZONE (not used in get_zone)
    2: 'sandybrown',     # EXCAVATION_ZONE
    3: 'lightgreen',     # DUMP_ZONE
    4: 'tan'             # BERM_ZONE
}

# Create a grid representation
grid = np.zeros((MAP_HEIGHT, MAP_WIDTH))

# Fill the grid with zone values
for y in range(MAP_HEIGHT):
    for x in range(MAP_WIDTH):
        grid[y, x] = get_zone(x, y)

# Plot the grid as an image - flipping the y-axis as requested
plt.imshow(grid, origin='upper', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT], 
           cmap=mcolors.ListedColormap(list(colors.values())), 
           vmin=-1, vmax=4)

# Create a legend
legend_patches = [Rectangle((0, 0), 1, 1, color=colors[i], label=name) 
                 for i, name in zip([-1, 0, 2, 3, 4], 
                                   ['Out of Bounds', 'Start Zone', 'Excavation Zone', 'Dump Zone', 'Berm Zone'])]
plt.legend(handles=legend_patches, loc='upper right')

ROBOT_WIDTH = 71
ROBOT_LENGTH = 98
ROBOT_RADIUS = math.sqrt((ROBOT_WIDTH/2)**2 + (ROBOT_LENGTH/2)**2) / 2

# Define example zones to visualize the Zone class points
berm_zone = Zone(428.0, 265.5, 70.0, 200.0)
print(f"height before: {berm_zone.height}")
berm_zone.shrink(ROBOT_RADIUS/2)
berm_zone.shrink(ROBOT_RADIUS, only_top=True)  # there might be no need to shrink berm since it already avoids parts near rocks
print(f"height after: {berm_zone.height}")
excavation_zone = Zone(0, 244.0, 274.0, 243.0)
excavation_zone.shrink(ROBOT_RADIUS)
start_zone = Zone(348.0, 0.0, 200.0, 200.0)

zones = [
    ("Berm", berm_zone),
    ("Excavation", excavation_zone)
]

# Generate and plot points for each zone
for name, zone in zones:
    # Create a Zone object - following the approach in zone.py
    points = []
    while not zone.is_done():
        p = zone.pop_next_point()
        points.append((p.x, p.y))
    points.reverse()
    
    # Plot the zone boundary
    rect = Rectangle((x, y), zone.width, zone.height, fill=False, edgecolor='red', linestyle='--', linewidth=2)
    ax.add_patch(rect)
    
    # Calculate total number of points and assign colors based on position in the list
    total_points = len(points)
    # Use a colormap to represent the order - viridis goes from purple (early in list) to yellow (late in list)
    colormap = cm.get_cmap('viridis')
    
    # Plot the points with colors representing their position in the zone.points list
    for idx, (px, py) in enumerate(points):
        # Normalize index to [0,1] range for coloring
        normalized_idx = 1 - (idx / total_points)  # Reverse so first points are yellow (more visible)
        plt.scatter(px, py, color=colormap(normalized_idx), s=10)
    
    # Add colorbar to show the progression
    sm = plt.cm.ScalarMappable(cmap=colormap)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label(f'Point ordering in {name} Zone')
    
    # Indicate the center
    center_x, center_y = zone.get_center().x, zone.get_center().y
    plt.scatter(center_x, center_y, color='red', s=50, marker='x')
    plt.text(center_x, center_y + 10, f"Center", ha='center')

# Labels and title
plt.title('Lunabotics Competition Map Visualization')
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.grid(True, alpha=0.3)

# Set axis limits
plt.xlim(0, MAP_WIDTH)
plt.ylim(MAP_HEIGHT, 0)  # Flipped y-axis

plt.tight_layout()
plt.show()
