import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from lunabotics_interfaces.action import SelfDriver, Plan
from lunabotics_interfaces.msg import Point
from lunabotics_interfaces.srv import Path

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event

# Goals:
# 1. Travel to the excavation zone and dig
# 2. Travel to the berm and dump the regolith
# 3. Repeat step 2

# We want to maximize berm volume
# We want to minimize:
# 1. Time
#    a. Wait time (such as LiDAR scanning or waiting for digging)
#    b. Travel time (such as waiting for the robot to travel to the target)
# 2. Watt hours
#    a. Motor usage


# each dump in the berm is about 10x10 cm
# TODO: use actual values
BERM_DUMP_SIZE = 10
ROBOT_WIDTH = 71
ROBOT_LENGTH = 98
LIDAR_VIEW_DISTANCE = 100
HAS_REACHED_TARGET_THRESHOLD = 10

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


# define zones in cm and shrink them so the robot doesn't hit the walls
berm_zone = Zone(428.0, 265.5, 70.0, 200.0)
berm_zone.shrink(ROBOT_WIDTH/2, only_top=True)  # there might be no need to shrink berm since it already avoids parts near rocks
excavation_zone = Zone(0, 244.0, 274.0, 243.0)
excavation_zone.shrink(ROBOT_WIDTH/2)
start_zone = Zone(348.0, 0.0, 200.0, 200.0)


class Planner(Node):

    def __init__(self, pathfinder_client):
        super().__init__('planner')
        self.odometry_action_client = ActionClient(self, SelfDriver, 'self_driver')
        self.planner_action_server = ActionServer(self, Plan, 'plan', self.plan_callback)
        self.goal_handle = None
        self.feedback_msg = None
        
        self.start = start_zone.get_center()
        self.previous_position = self.start
        self.current_target = None
        self.was_travel_successful = False 
        self.drive_time = 0
        self.callback_group = ReentrantCallbackGroup()
        self._travel_done_event = Event()
        
        self.pathfinder_client = pathfinder_client
    
    # gets run whenever a new goal is sent to the planner
    def plan_callback(self, goal_handle):
        self.goal_handle = goal_handle
        self.feedback_msg = Plan.Feedback()
        self.feedback_msg.progress = 0.0
        goal_handle.publish_feedback(self.feedback_msg)
        
        start_time = self.get_clock().now()
        should_excavate = goal_handle.request.should_excavate
        should_dump = goal_handle.request.should_dump
        start = goal_handle.request.start
        self.get_logger().info(f"Should excavate: {should_excavate}, should dump: {should_dump}")
        zone = None
        if should_excavate:
            zone = excavation_zone
        elif should_dump:
            zone = berm_zone
        else:
            self.get_logger().info("No excavation or dump zone specified")
            goal_handle.abort()
            result = Plan.Result()
            result.time_elapsed_millis = 0
            return
        self.current_target = zone.pop_next_point()
        self.get_logger().info(f"zone: {zone.x}, {zone.y}, {zone.x + zone.width}, {zone.y + zone.height} n = {zone.n}")
        self.get_logger().info(f"Current target: {self.current_target.x}, {self.current_target.y}")
        self.start = start

        self.was_travel_successful = False
        # keep looping until it finds a point that it can travel to
        self.drive_time = 0
        should_travel = self.travel_to_target()
        if not should_travel:
            goal_handle.abort()
            result = Plan.Result()
            current_time = self.get_clock().now()
            result.time_elapsed_millis = (current_time - start_time).nanoseconds // 1000000
            return result
        
        # check for cancellation
        while not self.was_travel_successful:
            if goal_handle.is_cancel_requested:
                self._travel_done_event.set()
                goal_handle.canceled()
                result = Plan.Result()
                current_time = self.get_clock().now()
                result.time_elapsed_millis = (current_time - start_time).nanoseconds // 1000000
                return result
        
        goal_handle.succeed()
        result = Plan.Result()
        current_time = self.get_clock().now()
        result.time_elapsed_millis = (current_time - start_time).nanoseconds // 1000000
        self.get_logger().info("Done. Planner took " + str(result.time_elapsed_millis - self.drive_time) + " ms to plan.")
        return result

    def send_drive_goal(self, targets):
        goal_msg = SelfDriver.Goal()
        goal_msg.targets = targets

        self.odometry_action_client.wait_for_server()
        self._send_goal_future = self.odometry_action_client.send_goal_async(goal_msg, feedback_callback=self.driving_feedback_callback)
        self._send_goal_future.add_done_callback(self.drive_goal_response_callback)

    def drive_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Traveling was rejected")
            return
        # wait for the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_drive_result_callback)
        
    def get_drive_result_callback(self, future):
        result = future.result().result
        time_elapsed = result.time_elapsed_millis
        self.get_logger().info(f"Travel to ({self.current_target.x}, {self.current_target.y}) took {time_elapsed} ms")

        self.drive_time += time_elapsed
        self._travel_done_event.set()
        self.travel_to_target()
        
    def driving_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # TODO: What if the time limit ends? we need to cancel the goal
        # TODO: If the robot isn't making very much progress, maybe cancel this goal and try a different route?
        progress = feedback.progress
        # self.get_logger().info(f"Progress to ({self.current_target.x}, {self.current_target.y}): {progress}")
    
    def make_path(self, target):
        self.get_logger().info(f"making path from {self.previous_position} to {target}")
        return use_pathfinder(self.pathfinder_client, self.previous_position, target)
    
    def prune_path(self, path):
        # get rid of all points in path that are further than 1 meter
        new_path = path.copy()
        sum_of_distances = 0
        for i in range(len(new_path) - 1):
            sum_of_distances += distance(new_path[i], new_path[i+1])
            if sum_of_distances > LIDAR_VIEW_DISTANCE:
                new_path = new_path[:i + 1]
                break
        return new_path
    
    # travel_to_target only returns true if the robot should continue
    # towards its final destination on its current trajectory
    def travel_to_target(self):
        # make new path
        path = self.make_path(self.current_target)
        # self.get_logger().info(f"Path: {path}")
        # check if path is valid, if not this travel was not successful, and we need to test a different point
        if len(path) <= 0:
            if distance(self.previous_position, self.current_target) < HAS_REACHED_TARGET_THRESHOLD:
                # now we have reached the destination
                self.get_logger().info(f"REACHED TARGET: {self.current_target.x}, {self.current_target.y}")
                self.was_travel_successful = True
                self.feedback_msg.progress = 1.0
                self.goal_handle.publish_feedback(self.feedback_msg)
                return True
            else:
                return False
        # we don't know about obstacles that are too far away, so we need to prune those points from the path
        pruned_path = self.prune_path(path)
        # send_drive_goal() will eventually call this function again through a callback
        self.send_drive_goal(pruned_path)
        # update the previous position so we know where we are for the next leg of the journey
        self.previous_position = pruned_path[-1]
        # update with feedback on the progress its made
        total_distance_to_target = distance(self.start, self.current_target)
        self.feedback_msg.progress = 1.0 - distance(self.previous_position, self.current_target) / total_distance_to_target
        self.goal_handle.publish_feedback(self.feedback_msg)
        return True


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


def use_pathfinder(pathfinder_client, start, end):
    future = pathfinder_client.make_path(start, end)
    # https://docs.ros.org/en/jazzy/How-To-Guides/Sync-Vs-Async.html
    # TODO: Maybe this will cause deadlock
    rclpy.spin_until_future_complete(pathfinder_client, future)
    response = future.result()
    # pathfinder_client.get_logger().info("Pathfinder response:" + str(response.nodes))
    return response.nodes


def main(args=None):
    rclpy.init(args=args)
    pathfinder_client = PathfinderClient()
    planner = Planner(pathfinder_client)
    executor = MultiThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(pathfinder_client)
    try:
        executor.spin()
    finally:
        planner.destroy_node()
        pathfinder_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()