import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


# the self driver agent action
from lunabotics_interfaces.msg import Point, Motors
from lunabotics_interfaces.action import SelfDriver
from navigation.pathfinder_helper import distance


from std_msgs.msg import Float32
import math


class Odometry(Node):
    """
        This node has an action server that takes requests to move the robot.
        It keeps track of the robot's position and orientation and wait until the robot reaches the requested position.
    """
    
    def __init__(self):
        super().__init__('odometry')

        # idk you need it for some reason
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            SelfDriver,
            'self_driver',
            execute_callback=self.on_goal_request,
            callback_group=self.callback_group
        )

        
        # action values to store for when its done
        self.goal_handle = None

        # fields
        # margin of error for rotation (degrees)
        self.deg_tolerance = 1.0 
        # margin of error for movement (centimeters)
        self.cm_tolerance = 5.0

        # initalize position
        self.position = Point()
        self.position.x = 448
        self.position.y = 100

        # initalize orientation
        self.orientation = 0.0

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
        # start the goal
        self.goal_handle = goal_handle
        feedback_msg = SelfDriver.Feedback()
        feedback_msg.progress = 0.0
        start_time = self.get_clock().now()

        # get points
        points = goal_handle.request.targets
        self.get_logger().info('received goal with {0} points'.format(len(points)))

        await self.to_path(points, goal_handle, feedback_msg)

        # send the result
        goal_handle.succeed()
        result = SelfDriver.Result()

        # find total time used to drive
        end_time = self.get_clock().now()
        result.time_elapsed_millis = (end_time - start_time).nanoseconds // 1_000_000
        return result

    # when position updates
    def on_position(self, position):
        # self.get_logger().info(f"pos: {position.x}, {position.y}")
        self.set_position(position)

    # when orientation updates
    def on_orientation(self, orientation):
        # self.get_logger().info(f"orient: {orientation.data}")
        self.set_orientation(orientation.data)

    # enforce orientation (no invariant enforcement yet)
    def set_position(self, position: Point):
        self.position = position

    # enforce orientation between 0 and 360
    def set_orientation(self, orientation: float):
        if (orientation < 0.0 or orientation > 360.0):
            self.get_logger().info(f"invalid orientation: {orientation}")
            return
        self.orientation = orientation

    # set the power of left and right front motors
    def set_motor_power(self, left_power: float, right_power: float):
        # ensure motor power between -1.0 and 1.0
        if (abs(left_power) > 1.0 or abs(right_power) > 1.0):
            self.get_logger.info(f"motor power must be between -1.0 and 1.0: L({left_power}), R({right_power})")
            return

        # message class
        msg = Motors()
        msg.front_left_wheel = left_power
        msg.front_right_wheel = right_power
        msg.back_left_wheel = left_power
        msg.back_right_wheel = right_power

        # not my business
        msg.conveyor = 0.0
        msg.outtake = 0.0

        # then publish it like usual
        self.motor_pub.publish(msg)
        # can save current motor power for future reference

    # orient the robot (global orientation)
    async def to_orient(self, final_orientation: float):
        # signed gap between desired and current orientation
        delta_orientation = self.orientation - final_orientation

        # continue to rotate toward target orientation until close enough
        while abs(delta_orientation) > self.deg_tolerance:
            delta_orientation = self.orientation - final_orientation
            # delta_orientation/45 means full power when gap > 45
            # calculate left motor power
            left_power = clamp(delta_orientation/45.0, -1.0, 1.0)
            # calculate right motor power
            right_power = -left_power
            # set motor power
            self.set_motor_power(left_power, right_power)
            #self.get_logger().info(f"orientation {self.orientation}, target {final_orientation}
            #self.get_logger().info(f"left {left_power}, right {right_power}")
            
            # wait 0.05 seconds; TODO: make a helper?
            future = rclpy.task.Future()
            self.create_timer(0.05, lambda: future.set_result(True))
            await future
            
        # stop the motors
        self.set_motor_power(0.0, 0.0)
        self.get_logger().info(f"cut motors, final orientation: {self.orientation}")

    # drive forward in a straight line
    def to_line(self, length: float):
        # get the normalized vector of where we face
        new_x = self.position.x + (math.cos(math.radians(self.orientation)) * length)
        new_y = self.position.y + (math.sin(math.radians(self.orientation)) * length)

        # calculate the x, y we end up at if driving straight, trust we get there
        self.to_position(new_x, new_y)

    # TODO: THIS BE THE PROBLEMATIC FUNCTION
    # move the robot from current to new position
    async def to_position(self, x: float, y: float):
        
        # calculate orientation to face position
        self.get_logger().info(f"want to go to position {x}, {y}")
        await self.face_position(x, y)
        self.get_logger().info(f"finished facing portion to {x}, {y}")
        
        # drive in a line until we reach the position
        target_point = Point(x=x, y=y)
        initial_position = self.position
        # delta x and delta y to get to the target position
        to_target = Point(x=target_point.x - initial_position.x, y=target_point.y - initial_position.y)
        
        # TODO: works perfectly, except passes a bit
        def passed_target_predicate():
            to_current = (self.position.x - initial_position.x, self.position.y - initial_position.y)
            # Dot product
            dot_product = to_target.x * to_current.x + to_target.y * to_current.y
            # Squared distance to target
            target_dist_squared = to_target.x**2 + to_target.y**2
            # If dot product < 0, robot is moving away from target
            # If dot product > target_dist_squared, robot has passed the target
            return dot_product < 0 or dot_product > target_dist_squared
        
        
        # Wait until we reach the point
        while not passed_target_predicate():
            # calculate the orientation we must be at to face this orientation
            target_orientation = math.atan2(target_point.y - self.position.y, target_point.x - self.position.x)
            
            # calculate orientation difference (normalized to [-pi, pi])
            diff = ((target_orientation - self.orientation + 180) % 360) - 180
            
            # base power (when no corrections to orientation) 
            base_power = 0.8

            # difference between current and ending position
            diff_cm = distance(target_point.x - self.position.x, target_point.y - self.position.y)

            # adjust power based on deviation (reduce power on the side we need to turn toward)
            left_power = clamp(diff_cm, -1.0, 1.0) #base_power + diff/45
            right_power = clamp(diff_cm, -1.0, 1.0) #base_power - diff/45
            
            # clamp power between [-1, 1]
            #left_power = clamp(left_power, -1.0, 1.0)
            #right_power = clamp(right_power, -1.0, 1.0)
            
            # set motor power
            self.set_motor_power(left_power, right_power)

            # use delayed rclpy.create_task() to wait / sleep
            future = rclpy.task.Future()
            self.create_timer(0.05, lambda: future.set_result(True))
            await future

        self.set_motor_power(0.0, 0.0)

    # orient the rover to face a position
    async def face_position(self, x: float, y: float):
        self.get_logger().info(f"want to face {x}, {y}")
        angle_rad = math.atan2(- y + self.position.y, x - self.position.x)
        new_orientation = positive_angle(math.degrees(angle_rad))

        # rotate to new orientation
        old_orientation = self.orientation
        await self.to_orient(new_orientation)
        self.get_logger().info(f"ok, facing {x}, {y}; old {old_orientation}; new {self.orientation}; desired {new_orientation}")

    # navigate along an entire path worth of coordinates (points)
    async def to_path(self, points, goal_handle, feedback_msg):
        points_len = len(points)
        
        # loop through each point and go there on the path
        # we enumerate to keep track of iterations
        for i, point in enumerate(points):
            await self.to_position(point.x, point.y)

            # publish progress traveling the path
            # e.g. 1 / 2 meaning 1 point reached out of 2 so far
            feedback_msg.progress = (i + 1) / points_len
            goal_handle.publish_feedback(feedback_msg)


# clamp a value between a range
# should replace with sigmoid function later
def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

# helper to ensure angle is its positive counterpart
def positive_angle(angle: float):
    return (angle + 360.0) % 360.0

# initialize odometry in our main
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
