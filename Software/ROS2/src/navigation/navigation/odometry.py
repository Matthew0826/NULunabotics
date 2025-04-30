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
        self.position = position

    # when orientation updates
    def on_orientation(self, orientation):
        # self.get_logger().info(f"orient: {orientation.data}")
        self.set_orientation(orientation.data)

    # enforce orientation between 0 and 360
    def set_orientation(self, orientation: float):
        if (orientation < 0 or orientation > 360):
            self.get_logger().info(f"invalid orientation: {orientation}")
            return
        self.orientation = orientation

    # set the power of left and right front motors
    def set_motor_power(self, left_power: float, right_power: float):
        # message class
        msg = Motors()
        msg.front_left_wheel = left_power
        msg.front_right_wheel = right_power
        msg.back_left_wheel = left_power
        msg.back_right_wheel = right_power
        msg.conveyor = 0.0
        msg.outtake = 0.0

        # then publish it like usual
        self.motor_pub.publish(msg)
        # can save current motor power for future reference

    # orient the robot (global orientation)
    async def to_orient(self, final_orientation: float):
        # margin of error for rotation to stop (degrees)
        tolerance = 10
        delta_orientation = final_orientation - self.orientation
        while abs(delta_orientation) > tolerance:
            delta_orientation = final_orientation - self.orientation
            # delta_orientation/90 means full power when gap > 90
            # calculate right motor power
            right_power = clamp(delta_orientation/45, -1.0, 1.0)
            # calculate left motor power
            left_power = -right_power
            # set motor power
            self.set_motor_power(left_power, right_power)
            #self.get_logger().info(f"orientation {self.orientation}, target {final_orientation}")
            #self.get_logger().info(f"delta {delta_orientation}, orient {self.orientation}, left {left_power}, right {right_power}")
            
            # wait 0.05 seconds
            future = rclpy.task.Future()
            self.create_timer(0.05, lambda: future.set_result(True))
            await future
            
        # stop the motors
        self.get_logger().info(f"stopping motors, final orientation: {self.orientation}")
        self.set_motor_power(0.0, 0.0)

    # drive forward in a straight line
    def to_line(self, length: float):
        # get the normalized vector of where we face
        new_x = self.position.x + (math.cos(math.radians(self.orientation)) * length)
        new_y = self.position.y + (math.sin(math.radians(self.orientation)) * length)

        # calculate the x, y we end up at if driving straight
        self.to_position(new_x, new_y)

    # move the robot from current to new position
    async def to_position(self, x: float, y: float):
        # calculate orientation to face position
        self.get_logger().info(f"going to {x}, {y}")
        await self.face_position(x, y)
        self.get_logger().info(f"OHH NO facing {x}, {y}")
        
        # drive in a line until we reach the position
        target_point = Point(x=x, y=y)
        initial_position = self.position
        to_target = (target_point.x - initial_position.x, target_point.y - initial_position.y)
        
        def passed_target_predicate():
            to_current = (self.position.x - initial_position.x, self.position.y - initial_position.y)
            # Dot product
            dot_product = to_target[0] * to_current[0] + to_target[1] * to_current[1]
            # Squared distance to target
            target_dist_squared = to_target[0]**2 + to_target[1]**2
            # If dot product < 0, robot is moving away from target
            # If dot product > target_dist_squared, robot has passed the target
            return dot_product < 0 or dot_product > target_dist_squared

        
        # Wait until we reach the point
        while not passed_target_predicate():
            target_orientation = math.atan2(target_point.y - self.position.y, target_point.x - self.position.x)
            # Calculate orientation difference (normalized to [-pi, pi])
            diff = ((target_orientation - self.orientation + 180) % 360) - 180
            # Base power
            base_power = 0.8
            # Adjust power based on deviation (reduce power on the side we need to turn toward)
            left_power = 1.0 #base_power + diff/45
            right_power = 1.0 #base_power - diff/45
            
            # clamp power to [-1, 1]
            left_power = clamp(left_power, -1.0, 1.0)
            right_power = clamp(right_power, -1.0, 1.0)
            
            # set motor power
            self.set_motor_power(left_power, right_power)
            # Use rclpy.create_task() + sleep via a future-like way
            future = rclpy.task.Future()
            self.create_timer(0.05, lambda: future.set_result(True))
            await future
        self.set_motor_power(0.0, 0.0)
    
    def positive_angle(self, angle: float):
        return (angle + 360) % 360

    # orient the rover to face a position
    async def face_position(self, x: float, y: float):
        self.get_logger().info(f"going to be facing {x}, {y}")
        angle_rad = math.atan2(- y + self.position.y, x - self.position.x)
        new_orientation = self.positive_angle(math.degrees(angle_rad))

        # rotate to new orientation
        old_orientation = self.orientation
        await self.to_orient(new_orientation)
        self.get_logger().info(f"i got to my orientation! my old one was {old_orientation}, now its {self.orientation}. i wanted {new_orientation}")

    async def to_path(self, points, goal_handle, feedback_msg):
        total_points = len(points)
        self.get_logger().info("hello")
        for i, point in enumerate(points):
            await self.to_position(point.x, point.y)

            feedback_msg.progress = (i + 1) / total_points
            goal_handle.publish_feedback(feedback_msg)


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
