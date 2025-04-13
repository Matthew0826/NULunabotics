from lunabotics_interfaces.msg import Point
# is each dump in the berm is about 10x10 cm?
BERM_DUMP_SIZE = 10

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
        # represents how far into the zone we are
        # as the robot digs into the excavation zone, it should pick a different place to dig from
        # start at end
        self.n = int(self.width * self.height / (BERM_DUMP_SIZE**2)) - 1
        self.points = []
        for i in range(int(width / BERM_DUMP_SIZE) + 1):
            for j in range(int(height / BERM_DUMP_SIZE) + 1):
                self.points.append(Point())
                self.points[-1].x = float(x + i * BERM_DUMP_SIZE)
                self.points[-1].y = float(y + j * BERM_DUMP_SIZE)

    def pop_next_point(self):
        point = self.points[self.n]
        self.n -= 1
        return point
    
    def shrink(self, amount, only_top=False):
        """Shrinks the zone by the given amount. If only_top is true, only the top of the zone is shrunk."""
        if not only_top:
            self.x += amount
            self.width -= amount * 2
        self.y += amount
        self.height -= amount * 2
        if only_top:
            self.height += amount
    
    def is_done(self):
        return self.n < 0
    
    def get_center(self):
        center_x = self.x + self.width / 2
        center_y = self.y + self.height / 2
        point = Point()
        point.x = float(center_x)
        point.y = float(center_y)
        return point

