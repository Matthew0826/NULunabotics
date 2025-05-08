import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup


# the self driver agent action
from lunabotics_interfaces.msg import Point, Motors, Excavate, PathVisual
from lunabotics_interfaces.action import SelfDriver
from navigation.pathfinder_helper import distance
from sensors.spin_node_helper import spin_nodes

from navigation.pid import PIDController


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

        # margin of error for rotation (degrees)
        self.deg_tolerance = 3.0 
        # margin of error for movement (centimeters)
        self.dist_tolerance = 15.0

        # initalize position
        self.position = Point()
        self.position.x = 448
        self.position.y = 100

        # initalize orientation
        self.orientation = 0.0
        self.last_orientation_time = self.get_clock().now()
        
        # dummy timer to keep event loop alive :)
        # self.create_timer(0.05, lambda: None)

        # ros publishers and subscribers
        self.motor_pub = self.create_publisher(Motors, 'physical_robot/motors', 10)
        self.excavate_pub = self.create_publisher(Excavate, 'physical_robot/excavate', 10)
        self.position_sub = self.create_subscription(Point, 'sensors/position', self.on_position, 10)
        self.orientation_sub = self.create_subscription(Float32, 'sensors/orientation', self.on_orientation, 10)
        self.website_path_visualizer = self.create_publisher(PathVisual, 'navigation/odometry_path', 10)
        
        # PID controllers (some notes from ChatGPT:)
        # Kp: Proportional Gain
            #  Corrects based on the current error.
            #  High Kp = fast response, but may overshoot or oscillate.
            #  Low Kp = slower reaction, smoother approach.
        # Ki: Integral Gain
            # Corrects based on the sum of past errors (helps eliminate bias/drift).
            # High Ki = aggressive accumulation (can cause instability).
            # Low/0 Ki = ignores long-term drift (safer for short-term motion like rotation).
        # Kd: Derivative Gain
            # Corrects based on the rate of change of error (slows down before overshooting).
            # High Kd = dampens oscillations, slows final approach.
            # Too much = system becomes unresponsive.
        self.orientation_pid = PIDController(Kp=0.015, Ki=0.0005, Kd=0.002, output_limits=(-0.8, 0.8))
        self.linear_drive_pid = PIDController(Kp=0.03, Ki=0.001, Kd=0.01, output_limits=(-0.8, 0.8))
        self.angular_drive_pid = PIDController(Kp=0.008, Ki=0.0, Kd=0.0, output_limits=(-0.5, 0.5))

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
        """Called when a new goal is sent to the action server. Drives to the requested position and measures how long it took."""
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
        self.set_position(position)

    # when orientation updates
    def on_orientation(self, orientation):
        self.set_orientation(orientation.data)

    def set_position(self, position: Point):
        """Called by the ROS subscriber."""
        self.position = position

    def set_orientation(self, orientation: float):
        """Called by the ROS subscriber. Enforces orientation between 0 and 360 degrees."""
        if (orientation < 0.0 or orientation > 360.0):
            self.get_logger().info(f"invalid orientation: {orientation}")
            return
        self.orientation = orientation
        
        delta_time = (self.get_clock().now() - self.last_orientation_time).nanoseconds / 1_000_000_000
        hz = 1.0/delta_time
        if hz < 5:
            self.get_logger().info(f"lagging! {hz}hz")
        self.last_orientation_time = self.get_clock().now()

    def set_motor_power(self, left_power: float, right_power: float):
        """Sets the power of left and right motors."""
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

        # then publish it like usual
        self.motor_pub.publish(msg)
        # can save current motor power for future reference

    # set the power of the conveyor and outtake motors on the excavator
    def set_excavate_power(self, elevator_power: float, conveyor_power: float, outtake_power: float):
        """Set the power of the excavator motors."""
        # ensure motor power between -1.0 and 1.0
        if (abs(elevator_power) > 1.0 or abs(outtake_power) > 1.0 or abs(outtake_power) > 1.0):
            self.get_logger.info(f"motor power must be between -1.0 and 1.0")
            return

        # message class (cannot move and excavate)
        msg = Excavate()
        msg.elevator = elevator_power
        msg.conveyor = conveyor_power
        msg.outtake = outtake_power

        # then publish it like usual
        self.excavate_pub.publish(msg)
        
    def get_degrees_error(self, final_degrees: float):
        """Calculates the difference in degrees between the robot's current orientation
            and a desired orientation."""
        # find the difference between the desired and current orientation
        return ((final_degrees - self.orientation + 180) % 360) - 180

    async def drive(self, left_power: float, right_power: float, seconds: float):
        """Sets the motor power for a given duration."""
        # set the motor power
        self.set_motor_power(left_power, right_power)
        # wait before next update
        future = rclpy.task.Future()
        def set_once():
            future.set_result(True)
            timer.cancel()  # clean up to prevent growth
        timer = self.create_timer(seconds, set_once)
        await future
    
    async def yield_once(self, delay: float = 0.01):
        future = rclpy.task.Future()
        def set_ready():
            future.set_result(True)
            timer.cancel()
        timer = self.create_timer(delay, set_ready)
        await future

    def get_orientation_error(self, destination: Point):
        """Calculates the difference in degrees between the robot's current orientation
            and the direction to a given destination."""
        dx = destination.x - self.position.x
        dy = self.position.y - destination.y
        desired_theta = math.degrees(math.atan2(dy, dx))
        return self.get_degrees_error(desired_theta)
    
    async def to_orient(self, final_orientation: float):
        """Rotates to a given orientation with PID control."""
        # reset the PID controller
        self.orientation_pid.reset()
        
        while True:
            # find error for PID
            error = self.get_degrees_error(final_orientation)
            # check if completed
            if abs(error) <= self.deg_tolerance: break
            # update PID
            power = self.orientation_pid.update(error, self.get_clock().now())
            # make sure theres at least a little power so the robot continues turning
            if abs(power) < 0.1:
                power = 0.1 if power > 0 else -0.1
            # drive the motors for a bit
            await self.drive(-power, power, seconds=0.2)
            await self.yield_once()

        # stop the motors
        self.set_motor_power(0.0, 0.0)
        self.get_logger().info(f"rotated to {int(self.orientation)} degrees. (wanted {int(final_orientation)})")

    async def to_position(self, destination: Point):
        """Drives to a given destination with PID control."""
        start_time = self.get_clock().now()
        # calculate orientation to face position
        self.get_logger().info(f"want to go to position {destination.x}, {destination.y}")
        self.website_path_visualizer.publish(PathVisual(nodes=[self.position, destination]))
        await self.face_position(destination)
        after_orientation_time = self.get_clock().now()
        
        # reset the PID controllers
        self.linear_drive_pid.reset()
        self.angular_drive_pid.reset()
        
        while True:
            # find error
            error = distance(self.position, destination)
            orientation_error = self.get_orientation_error(destination)
            # check if completed
            if error < self.dist_tolerance: break
            # update pid
            now = self.get_clock().now()
            forward_power = self.linear_drive_pid.update(error, now)
            turn_power = self.angular_drive_pid.update(orientation_error, now)
            # adjust motor power based on turn power from PID
            left_power = clamp(forward_power - turn_power, -1.0, 1.0)
            right_power = clamp(forward_power + turn_power, -1.0, 1.0)
            await self.drive(left_power, right_power, seconds=0.2)
            await self.yield_once()


        self.set_motor_power(0.0, 0.0)
        self.get_logger().info(f"drove to {int(self.position.x)}, {int(self.position.y)} (wanted {int(destination.x)}, {int(destination.y)})")
        after_moving_time = self.get_clock().now()
        # calculate time taken to move
        time_taken = (after_moving_time - start_time).nanoseconds // 1_000_000
        self.get_logger().info(f"took {time_taken} milliseconds to move")
        # calculate time taken to orient
        orientation_time = (after_orientation_time - start_time).nanoseconds // 1_000_000
        self.get_logger().info(f"took {orientation_time} milliseconds to orient")

    # orient the rover to face a position
    async def face_position(self, destination: Point):
        """Rotates to face a given position."""
        self.get_logger().info(f"want to face {destination.x}, {destination.y}")
        angle_rad = math.atan2(self.position.y - destination.y, destination.x - self.position.x)
        new_orientation = positive_angle(math.degrees(angle_rad))

        # rotate to new orientation
        old_orientation = self.orientation
        await self.to_orient(new_orientation)
        self.get_logger().info(f"faced position: old {int(old_orientation)}; new {int(self.orientation)}; desired {int(new_orientation)}")

    # navigate along an entire path worth of coordinates (points)
    async def to_path(self, points, goal_handle, feedback_msg):
        """Drives to a series of points in order, while reporting its progress over the goal handle."""
        points_len = len(points)
        
        # loop through each point and go there on the path
        # we enumerate to keep track of iterations
        for i, point in enumerate(points):
            await self.to_position(point)

            # publish progress traveling the path
            # e.g. 1 / 2 meaning 1 point reached out of 2 so far
            feedback_msg.progress = (i + 1) / points_len
            goal_handle.publish_feedback(feedback_msg)
            await self.yield_once()
        
        # empty list to show its done
        self.website_path_visualizer.publish(PathVisual(nodes=[]))


# should this be replaced with sigmoid function later?
def clamp(value, min_value, max_value):
    """Clamp a value between min_value and max_value."""
    return max(min_value, min(max_value, value))

# helper to ensure angle is its positive counterpart
def positive_angle(angle: float):
    return (angle + 360.0) % 360.0

# initialize odometry in our main
def main(args=None):
    rclpy.init(args=args)
    odometry = Odometry()
    spin_nodes(odometry, is_async=True)
    


if __name__ == '__main__':
    main()
