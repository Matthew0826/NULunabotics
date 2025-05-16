import math

BUCKET_SIZE = 0.5  # cm
VIEW_CONE_SIZE = 120  # cm

class MovingAverage:
    """A simple moving average class to keep track of the last N LiDAR rotations."""

    def __init__(self, size):
        self.size = size
        self.data = {}
        self.count = 0

    def add(self, points):
        """
        Adds a list of points to the moving average. Each point is a tuple of (distance, angle).
        Assumes points are distances in cm and angles in radians.
        """
        for point in points:
            x = point.distance * math.cos(point.angle)
            # find nearest bucket
            x = int(x / BUCKET_SIZE) * BUCKET_SIZE
            y = int(point.distance * math.sin(point.angle))
            if abs(x) > VIEW_CONE_SIZE:
                # ignore points outside the view cone
                continue
            if x not in self.data:
                self.data[x] = []
            if len(self.data[x]) >= self.size:
                # remove first point
                self.data[x] = self.data[x][1:]
            # add new point
            self.data[x].append(y)
        # track how many points we have added
        if self.count < self.size:
            self.count += 1
    
    def get_average_points(self):
        """Returns the average of the points in the moving average."""
        averages = []
        for x, y_list in self.data.items():
            if len(y_list) > 0:
                avg_y = sum(y_list) / len(y_list)
                averages.append((x, avg_y))
        return averages
    
    def is_full(self):
        return self.count >= self.size
    
    def clear(self):
        self.data = {}
        self.count = 0
    
    def debug_print(self):
        """Prints the current state of the moving average using ASCII art."""
        average_points = self.get_average_points()
        for x, y in average_points:
            x = int(x)
            y = int(y)
            print(f"{x}: {' ' * (y + 10) + '#'}")
