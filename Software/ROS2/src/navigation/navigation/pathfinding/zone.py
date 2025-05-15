from lunabotics_interfaces.msg import Point
from .pathfinder_helper import distance
# is each dump in the berm is about 10x10 cm?
BERM_DUMP_SIZE = 15

class Zone:
    """Represents a zone on the competition map. The zone is a rectangle with a width and height. Options are:
    - excavation zone
    - dump zone
    - obstacle zone
    - start zone
    - or out of bounds
    """
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.generate_points()
    
    def generate_points(self):
        # represents how far into the zone we are
        # as the robot digs into the excavation zone, it should pick a different place to dig from
        # start at end
        self.points = []
        for i in range(int(self.width / BERM_DUMP_SIZE) + 1):
            for j in range(int(self.height / BERM_DUMP_SIZE) + 1):
                p = Point()
                p.x = float(self.x + i * BERM_DUMP_SIZE)
                p.y = float(self.y + j * BERM_DUMP_SIZE)
                self.points.append(p)

    def pop_next_point(self):
        # find point closest to the center of the zone
        closest_point = None
        closest_distance = float('inf')
        center = self.get_center()
        for point in self.points:
            dist = distance(center, point)
            if dist < closest_distance:
                closest_distance = dist
                closest_point = point
        # remove the point from the list
        self.points.remove(closest_point)
        return closest_point
    
    def shrink(self, amount, only_top=False):
        """Shrinks the zone by the given amount. If only_top is true, only the top of the zone is shrunk."""
        if not only_top:
            self.x += amount
            self.width -= amount * 2
        self.y += amount
        self.height -= amount * 2
        if only_top:
            self.height += amount
        self.generate_points()
    
    def is_done(self):
        return len(self.points) == 0
    
    def get_center(self):
        center_x = self.x + self.width / 2
        center_y = self.y + self.height / 2
        point = Point()
        point.x = float(center_x)
        point.y = float(center_y)
        return point

