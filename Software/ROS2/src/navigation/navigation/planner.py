import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from lunabotics_interfaces.action import SelfDriver
from lunabotics_interfaces.msg import Point

# each dump in the berm is about 10x10 cm
# TODO: use actual value
BERM_DUMP_SIZE = 10
ROBOT_WIDTH = 71
ROBOT_LENGTH = 98  # units: cm
LIDAR_VIEW_DISTANCE = 100

def distance(point, other_point):
    return ((point.x - other_point.x) ** 2 + (point.y - other_point.y) ** 2) ** 0.5

class Zone:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        # represents how far into the zone we are
        # as the robot digs into the excavation zone, it should pick a different place to dig from
        # start at end
        self.n = (self.width * self.height // BERM_DUMP_SIZE // BERM_DUMP_SIZE) - 1
    
    def peek_nth_point(self):
        # get the nth point in the rectangle, split into grid sizes of BERM_DUMP_SIZE
        x = self.x + (self.n % self.width) * BERM_DUMP_SIZE
        y = self.y + (self.n // self.width) * BERM_DUMP_SIZE
        point = Point()
        point.x = float(x)
        point.y = float(y)
        return point

    def pop_next_point(self):
        point = peek_nth_point()
        self.n -= 1
        return point
    
    def shrink(self, amount, only_top=False):
        if !only_top:
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


# define zones in cm and shrink them so the robot doesnt hit the walls
berm_zone = Zone(428.0, 265.5, 70.0, 200.0)
berm_zone.shrink(ROBOT_WIDTH/2, only_top=True)  # there might be no need to shrink berm since it already avoids parts near rocks
excavation_zone = Zone(0, 244.0, 274.0, 243.0)
excavation_zone.shrink(ROBOT_WIDTH/2)
start_zone = Zone(348.0, 0.0, 200.0, 200.0)


class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        self._action_client = ActionClient(self, SelfDriver, 'self_driver')
        self.previous_position = start_zone.get_center()

    def send_goal(self, targets):
        goal_msg = SelfDriver.Goal()
        goal_msg.targets = targets

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
    
    def make_path(self, target):
        return use_pathfinder(self.previous_position, target)
    
    def prune_path(self, path):
         # get rid of all points in path that are further than 1 meter
        sum_of_distances = 0
        for i in range(len(path) - 1):
            sum_of_distances += distance(path[i], path[i+1])
            if sum_of_distances > LIDAR_VIEW_DISTANCE:
                path = path[:i]
                break
        return path
    
    def go_to(self, target):
        has_moved = False
        while True:
            # make new path
            path = self.make_path(target)
            # check if path is valid, and only return true if its moved at all
            if len(path) <= 0: return has_moved
            # we don't know about obstacles that are too far away, so we need to prune those points from the path
            path = self.prune_path(path)
            self.send_goal(path)
            has_moved = True
    
    def travel_to_zone(self, zone):
        success = False
        # keep looping until it finds a point that it can travel to
        while not success:
            success = self.go_to(zone.pop_next_point(self.berm_dump_count))
    
    def do_lunabotics_competition_plan(self):
        while not excavation_zone.is_done():
            self.travel_to_zone(excavation_zone)
            self.excavate()
            self.travel_to_zone(berm_zone)
            self.dump()

class PathfinderClient(Node):
    def __init__(self):
        super().__init__('pathfinder_client')
        self.pathfinder_client = self.create_client(Path, 'pathfinder')
        while not self.pathfinder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pathfinder service not available, waiting again...')
        self.req = Path.Request()
    
    def make_path(self, start, end):
        self.req.start = start
        self.req.end = end
        return self.pathfinder_client.call_async(self.req)


pathfinder_client = PathfinderClient()
def use_pathfinder(start, end):
    future = pathfinder_client.make_path(start, end)
    # https://docs.ros.org/en/jazzy/How-To-Guides/Sync-Vs-Async.html
    # TODO: Maybe this will cause deadlock
    rclpy.spin_until_future_complete(pathfinder_client, future)
    response = future.result()
    return response.path


def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    pathfinder_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()