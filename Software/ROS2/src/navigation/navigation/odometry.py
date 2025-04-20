import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from lunabotics_interfaces.msg import Point, Motors

# the self driver agent action
from lunabotics_interfaces.action import SelfDriver

from navigation.pathfinder_helper import distance
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from std_msgs.msg import Float32

import math
import time
import asyncio


# A subscriber to events which update the rover.
class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            SelfDriver,
            'self_driver',
            execute_callback=self.on_goal_request,  # now async!
            callback_group=self.callback_group
        )

        
        # action values to store for when its done
        self.goal_handle = None
        self.start_time = 0

        # initalize position
        self.position = Point()
        self.position.x = 448
        self.position.y = 100

        # initalize orientation
        self.orientation = 0.0
        
        # initialize motor power
        self.motor_power_left = 0.0
        self.motor_power_right = 0.0

        # stop predicate returns true when the robot should stop sending power to the motors
        self.stop_motors_predicate = lambda: True
        
        # these are functions that calculate the motor power
        self.left_power_calculator = lambda: 0.0
        self.right_power_calculator = lambda: 0.0
        self.previous_direction = 0

        # ros publishers and subscribers
        self.motor_pub = self.create_publisher(
            Motors,
            'physical_robot/motors',
            10
        )
        
        self.position_sub = self.create_subscription(
            Point,
            'sensors/position',
            self.on_position,
            10)
        
        self.orientation_sub = self.create_subscription(
            Float32,
            'sensors/orientation',
            self.on_orientation,
            10)
        
        self.timer = self.create_timer(0.05, self.timer_callback)

    # when someone calls this action
    # SelfDriver.action:
    # # Request
    # Point[] targets
    # ---
    # # Result
    # int64 time_elapsed_millis
    # ---
    # # Feedback
    # # from 0 to 1
    # float progress
    async def on_goal_request(self, goal_handle):
        self.goal_handle = goal_handle
        feedback_msg = SelfDriver.Feedback()
        feedback_msg.progress = 0.0
        self.start_time = self.get_clock().now()

        points = goal_handle.request.targets
        self.get_logger().info('Received goal with {0} points'.format(len(points)))

        await self.to_path(points, goal_handle, feedback_msg)

        return self.finish_goal()

    
    def finish_goal(self):
        # send the result
        self.goal_handle.succeed()
        result = SelfDriver.Result()
        # find total time used to drive
        end_time = self.get_clock().now()
        result.time_elapsed_millis = (end_time - self.start_time).nanoseconds // 1_000_000
        return result

    # when position updates
    def on_position(self, position):
        # self.get_logger().info(f"pos: {position.x}, {position.y}")
        self.position = position

    # when orientation updates
    def on_orientation(self, orientation):
        # self.get_logger().info(f"orient: {orientation.data}")
        self.set_orientation(orientation.data)

    # enforce orientation between 0 and 360
    def set_orientation(self, orientation: float):
        if (orientation < 0 or orientation > 360): 
            print(f"Invalid Orientation: {orientation}")
            return
        self.orientation = orientation

    # set the power of left and right front motors
    def set_motor_power(self, left_power, right_power):
        # message class
        msg = Motors()
        # one forth speed is plenty for now!
        msg.front_left_wheel = left_power
        msg.front_right_wheel = right_power
        msg.back_left_wheel = left_power
        msg.back_right_wheel = right_power
        msg.conveyor = 0.0
        msg.outtake = 0.0

        # Then publish it like usual
        self.motor_pub.publish(msg)
        # can save current motor power for future reference

    # orient the robot (global orientation)
    def to_orient(self, final_orientation: float):
        # Fix for the 90-degree offset issue
        # Adjust the target orientation by 90 degrees to compensate for the misalignment
        adjusted_final_orientation = (final_orientation) % 360
        
        # Calculate shortest path direction to the adjusted target
        def shortest_path_direction():
            diff = (adjusted_final_orientation - self.orientation) % 360 - 180
            return -1 if diff < 0 else 1  # -1 for left turn, 1 for right turn
        
        # Stop when direction changes, meaning we've gone past the target
        self.stop_motors_predicate = lambda: shortest_path_direction() != self.previous_direction
        
        # Update previous direction each time
        self.previous_direction = shortest_path_direction()
        
        # Power based on shortest direction
        self.left_power_calculator = lambda: -1.0 if shortest_path_direction() < 0 else 1.0
        self.right_power_calculator = lambda: -self.left_power_calculator()

    # drive forward in a straight line
    def to_line(self, length: float):
        # get the normalized vector of where we face
        new_x = self.position.x + (math.cos(math.radians(self.orientation)) * length)
        new_y = self.position.y + (math.sin(math.radians(self.orientation)) * length)

        # calculate the x, y we end up at if driving straight
        self.to_position(new_x, new_y)

    async def wait_until_destination(self):
        # Wait until we reach the point
        while not self.stop_motors_predicate():
            # Use rclpy.create_task() + sleep via a future-like way
            future = rclpy.task.Future()
            self.create_timer(0.05, lambda: future.set_result(True))
            await future

    # move the robot from current to new position
    async def to_position(self, x: float, y: float):
        # calculate orientation to face position
        self.get_logger().info(f"going to {x}, {y}")
        await self.face_position(x, y)
        
        # drive in a line until we reach the position
        target_point = Point(x=x, y=y)
        # Predicate that checks if we've passed the target along our path
        def passed_target_predicate():
            # Vector from initial to target
            to_target = (target_point.x - self.initial_position.x, target_point.y - self.initial_position.y)
            # Vector from initial to current position
            to_current = (self.position.x - self.initial_position.x, self.position.y - self.initial_position.y)
            # Dot product
            dot_product = to_target[0] * to_current[0] + to_target[1] * to_current[1]
            # Squared distance to target
            target_dist_squared = to_target[0]**2 + to_target[1]**2
            # If dot product < 0, robot is moving away from target
            # If dot product > target_dist_squared, robot has passed the target
            return dot_product < 0 or dot_product > target_dist_squared

        # Store initial position to use as reference
        self.initial_position = self.position
        
        def calculate_left_power():
            # Get ideal orientation to target
            target_orientation = math.atan2(target_point.y - self.position.y, target_point.x - self.position.x)
            # Calculate orientation difference (normalized to [-pi, pi])
            diff = (target_orientation - self.orientation + math.pi) % (2 * math.pi) - math.pi
            # Base power
            base_power = 0.8
            # Adjust power based on deviation (reduce power on the side we need to turn toward)
            if diff > 0:  # Need to turn left
                return -base_power * 0.8  # Reduce left power to turn left
            else:
                return -base_power * 1.2  # Increase left power to turn right
        
        def calculate_right_power():
            # Get ideal orientation to target
            target_orientation = math.atan2(target_point.y - self.position.y, target_point.x - self.position.x)
            # Calculate orientation difference (normalized to [-pi, pi])
            diff = (target_orientation - self.orientation + math.pi) % (2 * math.pi) - math.pi
            # Base power
            base_power = 0.8
            # Adjust power based on deviation (reduce power on the side we need to turn toward)
            if diff > 0:  # Need to turn left
                return -base_power * 1.2  # Increase right power to turn left
            else:
                return -base_power * 0.8  # Reduce right power to turn right
        
        self.stop_motors_predicate = passed_target_predicate
        self.left_power_calculator = calculate_left_power
        self.right_power_calculator = calculate_right_power
        await self.wait_until_destination()
            
    
    # called every 0.05 seconds
    def timer_callback(self):
        # turn off motors if the predicate returns true
        if self.stop_motors_predicate():
            self.motor_power_left = 0.0
            self.motor_power_right = 0.0
        else:
            self.motor_power_left = self.left_power_calculator()
            self.motor_power_right = self.right_power_calculator()
        # set motor power
        # self.get_logger().info(f"motor power: {self.motor_power_left}, {self.motor_power_right}")
        self.set_motor_power(self.motor_power_left, self.motor_power_right)
        

    # orient the rover to face a position
    async def face_position(self, x: float, y: float):
        angle_rad = math.atan2(y - self.position.y, x - self.position.x)
        new_orientation = math.degrees(angle_rad)

        # rotate to new orientation
        old_orientation = self.orientation
        while (self.orientation - new_orientation + 360) % 360 > 5:
            self.to_orient(new_orientation)
            await self.wait_until_destination()
        self.get_logger().info(f"i got to my orientation! my old one was {old_orientation}, now its {self.orientation}. i wanted {new_orientation}")

    async def to_path(self, points, goal_handle, feedback_msg):
        total_points = len(points)
        self.get_logger().info("hello")
        for i, point in enumerate(points):
            while distance(self.position, point) > 20:
                await self.to_position(point.x, point.y)

            feedback_msg.progress = (i + 1) / total_points
            goal_handle.publish_feedback(feedback_msg)


    # STRETCH: give a set of lines (graph) to drive along)
    # I think this would be a set of points actually
    def to_graph(self):
        pass

    # self = the class to add subscription to
    # getType = the type to send from the topic to subscriber
    # topicName = the name of the event to subscribe to
    # callbackName = the name of callback function on self to use
    # def subscribe(self, get_type, topic_name: str, callback_name: str):
    #     return self.create_subscription(
    #         get_type, 
    #         # the topic to subscribe t
    #         topic_name, 
    #         # the function called on an event
    #         self[callback_name],
    #         10,
    #         )

# clamp a value between a range
# should replace with sigmoid function later
def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    odometry = Odometry()

    executor = MultiThreadedExecutor()
    executor.add_node(odometry)

    try:
        executor.spin()
    finally:
        odometry.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
