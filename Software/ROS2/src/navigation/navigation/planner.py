import rclpy
from rclpy.action import ActionClient, ActionServer
from action_msgs.msg import GoalStatus
from rclpy.node import Node

from lunabotics_interfaces.action import SelfDriver, Plan

from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Event

from navigation.pathfinder_client import PathfinderClient, use_pathfinder
from navigation.zone import Zone
from navigation.pathfinder_helper import distance

from sensors.spin_node_helper import spin_nodes

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


# TODO: use actual values
LIDAR_VIEW_DISTANCE = 60 # cm
ROBOT_WIDTH = 71
ROBOT_LENGTH = 98
HAS_REACHED_TARGET_THRESHOLD = 20


# define zones in cm and shrink them so the robot doesn't hit the walls
berm_zone = Zone(428.0, 265.5, 70.0, 200.0)
berm_zone.shrink(ROBOT_WIDTH/2, only_top=True)  # there might be no need to shrink berm since it already avoids parts near rocks
excavation_zone = Zone(0, 244.0, 274.0, 243.0)
excavation_zone.shrink(ROBOT_WIDTH/2)
start_zone = Zone(348.0, 0.0, 200.0, 200.0)


class Planner(Node):
    """Plans and sends out commands to traverse a route between the robot's position and a target zone."""

    def __init__(self, pathfinder_client):
        super().__init__('planner')
        self.odometry_action_client = ActionClient(self, SelfDriver, 'self_driver')
        self.planner_action_server = ActionServer(self, Plan, 'plan', self.plan_callback, cancel_callback=self.cancel_callback)
        self.goal_handle = None
        self.feedback_msg = None
        self.odometry_goal_handle = None
        
        self.start = start_zone.get_center()
        self.previous_position = self.start
        self.current_target = None
        self.was_travel_successful = False 
        self.was_cancelled = False
        self.drive_time = 0
        self.callback_group = ReentrantCallbackGroup()
        
        self.pathfinder_client = pathfinder_client
    
    def cancel_callback(self, goal_handle):
        """
        Gets run whenever a goal is cancelled.
        This is used to stop the robot from moving if the plan is cancelled.
        """
        self.get_logger().info("Cancelling goal.")
        if self.odometry_goal_handle is not None:
            self.odometry_goal_handle.cancel_goal_async()
        self.was_cancelled = True
        return Plan.Result()
    
    async def plan_callback(self, goal_handle):
        """
        Gets run whenever a new goal is sent to the planner.
        The plan can either travel to the excavation zone or the dump zone.
        While waiting for it to be complete, the plan may be cancelled.
        Times how long it takes to travel to the target.
        """
        self.goal_handle = goal_handle
        self.feedback_msg = Plan.Feedback()
        self.feedback_msg.progress = 0.0
        goal_handle.publish_feedback(self.feedback_msg)
        
        start_time = self.get_clock().now()
        should_excavate = goal_handle.request.should_excavate
        should_dump = goal_handle.request.should_dump
        destination = goal_handle.request.destination
        start = goal_handle.request.start
        self.get_logger().info(f"Should excavate: {should_excavate}, should dump: {should_dump}")
        zone = None
        if destination.x >= 0 and destination.y >= 0:
            # make a fake zone because the destination was overridden
            zone = Zone(destination.x, destination.y, 1, 1)
        elif should_excavate:
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
        self.was_cancelled = False
        self.drive_time = 0

        while not self.was_travel_successful:
            if self.was_cancelled:
                self.get_logger().info("Goal was cancelled.")
                result = Plan.Result()
                result.time_elapsed_millis = (self.get_clock().now() - start_time).nanoseconds // 1000000
                return result
            
            should_continue = await self.travel_to_target()
            if not should_continue:
                goal_handle.abort()
                result = Plan.Result()
                result.time_elapsed_millis = (self.get_clock().now() - start_time).nanoseconds // 1_000_000
                return result
        
        
        # we are at the end, we need to dig/dump (no path to travel to)
        excavation_start_time = self.get_clock().now()
        await self.send_drive_goal([], should_excavate=should_excavate, should_dump=should_dump)
        excavation_end_time = self.get_clock().now()
        self.drive_time += (excavation_end_time - excavation_start_time).nanoseconds // 1_000_000
        self.get_logger().info(f"Excavation took {self.drive_time} ms")
        
        # "Je gagne!"
        goal_handle.succeed()
        result = Plan.Result()
        current_time = self.get_clock().now()
        result.time_elapsed_millis = (current_time - start_time).nanoseconds // 1000000
        self.get_logger().info("Done. Planner took " + str(result.time_elapsed_millis - self.drive_time) + " ms to plan.")
        return result

    async def send_drive_goal(self, targets, should_excavate=False, should_dump=False):
        """Sends a drive goal to the odometry action server, and waits for the response."""
        # make sure the action server is available
        self.odometry_action_client.wait_for_server()
        
        # send in this driving goal
        goal_msg = SelfDriver.Goal()
        goal_msg.targets = targets
        goal_msg.should_excavate = should_excavate
        goal_msg.should_unload = should_dump
        # for now we want to back up when we finished
        goal_msg.should_reverse = should_dump
        goal_handle = await self.odometry_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.driving_feedback_callback
        )
        self.odometry_goal_handle = goal_handle
        
        if not goal_handle.accepted:
            self.get_logger().warn("Drive goal rejected.")
            return False

        # drive goal was accepted, so we can wait for the result
        self.get_logger().info("Drive goal accepted.")
        res = await goal_handle.get_result_async()
        
        # proccess the result
        result = res.result
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # the drive goal was successful
            # so, count the time it took to drive
            time_elapsed = result.time_elapsed_millis
            self.drive_time += time_elapsed
            self.get_logger().info(f"Travel to ({self.current_target.x}, {self.current_target.y}) took {time_elapsed} ms")
            return True
        else:
            self.get_logger().warn("Drive failed.")
            return False
        
    def driving_feedback_callback(self, feedback_msg):
        """Gets called when the drive goal is in progress. 
        This is used to update the progress of the robot, as well as check if the robot is making suffiecient progress towards its goal."""
        feedback = feedback_msg.feedback
        # TODO: What if the time limit ends? we need to cancel the goal
        # TODO: If the robot isn't making very much progress, maybe cancel this goal and try a different route?
        progress = feedback.progress
        finished_driving = feedback.finished_driving
        # self.get_logger().info(f"Progress to ({self.current_target.x}, {self.current_target.y}): {progress}")
    
    def make_path(self, target):
        """Calls pathfinder from the pathfinder client to get a path to the target."""
        return use_pathfinder(self.pathfinder_client, self.previous_position, target)
    
    def prune_path(self, path):
        """Gets rid of all points in path that are further than LIDAR_VIEW_DISTANCE cms from the current position of the robot.
        This is because the robot shouldn't travel into territory it can't see yet."""
        new_path = path.copy()
        sum_of_distances = 0
        for i in range(len(new_path) - 1):
            sum_of_distances += distance(new_path[i], new_path[i+1])
            if sum_of_distances > LIDAR_VIEW_DISTANCE:
                new_path = new_path[:i + 1]
                break
        return new_path
    
    async def travel_to_target(self):
        """
        Makes a path to the target, then sends a drive goal to the odometry action server.
        If the path is empty, it either means the plan is complete, or the robot is stuck.
        Only returns true if the robot should continue towards its final destination on its current trajectory.
        """
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
        # remove first point so the robot doesn't drive to itself
        # then wait for the robot to drive to the next point
        drive_success = await self.send_drive_goal(pruned_path[1:])
        if not drive_success:
            return False
        # update the previous position so we know where we are for the next leg of the journey
        self.previous_position = pruned_path[-1]
        # update with feedback on the progress its made
        total_distance_to_target = distance(self.start, self.current_target)
        self.feedback_msg.progress = 1.0 - distance(self.previous_position, self.current_target) / total_distance_to_target
        self.goal_handle.publish_feedback(self.feedback_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    pathfinder_client = PathfinderClient()
    spin_nodes(Planner(pathfinder_client), pathfinder_client, is_async=True)


if __name__ == '__main__':
    main()