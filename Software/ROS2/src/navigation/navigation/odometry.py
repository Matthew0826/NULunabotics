import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup


# the self driver agent action
from lunabotics_interfaces.msg import Point, Motors, Excavator, PathVisual, ExcavatorPotentiometer
from lunabotics_interfaces.action import SelfDriver
from navigation.pathfinder_helper import distance
from sensors.spin_node_helper import spin_nodes

from navigation.pid import PIDController


from std_msgs.msg import Float32
import math

MAX_EXCAVATOR_PERCENT = 0.9
MIN_EXCAVATOR_PERCENT = 0.1

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
            cancel_callback=self.on_cancel,
            callback_group=self.callback_group
        )
        
        # action values to store for when its done
        self.goal_handle = None
        self.was_cancelled = False

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
        
        # initalize linear actuator percentage for the excavator
        # 0% is contracted and 100% is extended
        self.excavator_left_percent = 0.0
        self.excavator_right_percent = 0.0
        
        # dummy timer to keep event loop alive :)
        # self.create_timer(0.05, lambda: None)

        # ros publishers and subscribers
        self.motor_pub = self.create_publisher(Motors, 'physical_robot/motors', 10)
        self.excavate_pub = self.create_publisher(Excavator, 'physical_robot/excavator', 10)
        self.position_sub = self.create_subscription(Point, 'sensors/position', self.on_position, 10)
        self.orientation_sub = self.create_subscription(Float32, 'sensors/orientation', self.on_orientation, 10)
        self.excavator_percent_sub = self.create_subscription(ExcavatorPotentiometer, 'sensors/excavator_percent', self.on_excavator_percent, 10)
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
        # for some reason this one starts doing big loops around the target when you give it some Ki and Kd on the simulation
        # especially when the speed of the simulation is increased
        self.angular_drive_pid = PIDController(Kp=0.008, Ki=0.0, Kd=0.0, output_limits=(-0.35, 0.35))
        
        self.excavator_left_actuator_pid = PIDController(Kp=0.01, Ki=0.0, Kd=0.0, output_limits=(-1.0, 1.0))
        self.excavator_right_actuator_pid = PIDController(Kp=50.0, Ki=0.0, Kd=0.0, output_limits=(-1.0, 1.0))

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
        feedback_msg.finished_driving = False
        start_time = self.get_clock().now()
        self.was_cancelled = False

        # get points
        points = goal_handle.request.targets
        self.get_logger().info('received goal with {0} points'.format(len(points)))

        # drive in reverse if told by the goal
        await self.to_path(points, goal_handle, feedback_msg, goal_handle.request.should_reverse)
        
        # get if we should unload
        unload = goal_handle.request.should_unload
        if unload:
            self.get_logger().info('unloading')
            await self.run_unload_bin()
            self.get_logger().info('unloaded')
        
        # get if we should excavate
        excavate = goal_handle.request.should_excavate
        if excavate:
            self.get_logger().info('excavating')
            # start the excavator
            await self.move_excavator(MAX_EXCAVATOR_PERCENT)
            # start the conveyor
            await self.run_conveyor()
            await self.move_excavator(MIN_EXCAVATOR_PERCENT)
            self.get_logger().info('excavated')
        
        # send the result
        goal_handle.succeed()
        result = SelfDriver.Result()

        # find total time used to drive
        end_time = self.get_clock().now()
        result.time_elapsed_millis = (end_time - start_time).nanoseconds // 1_000_000
        return result

    def on_cancel(self, goal_handle):
        """Called when the action is cancelled. Stops the robot and sends a result."""
        self.get_logger().info('cancelled goal')
        # stop the motors
        self.set_motor_power(0.0, 0.0)
        # stop the excavator
        self.stop_excavator()
        result = SelfDriver.Result()
        result.time_elapsed_millis = 0
        self.was_cancelled = True
        return result

    # when position updates
    def on_position(self, position):
        self.set_position(position)

    # when orientation updates
    def on_orientation(self, orientation):
        self.set_orientation(orientation.data)
        
    # when excavator percent updates
    def on_excavator_percent(self, percent):
        self.set_excavator_percent(percent.left_actuator_percent, percent.right_actuator_percent)

    def set_position(self, position: Point):
        """Called by the ROS subscriber."""
        self.position = position
    
    def set_excavator_percent(self, left_percent: float, right_percent: float):
        """Called by the ROS subscriber. Enforces percent between 0 and 1."""
        if (left_percent < 0.0 or left_percent > 1.0 and right_percent < 0.0 or right_percent > 1.0):
            self.get_logger().info(f"invalid excavator percent")
            return
        self.excavator_left_percent = left_percent
        self.excavator_right_percent = right_percent

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
    # actuator_power: -1.0 is retract; 1.0 is extend
    def set_excavator_power(self, left_actuator_power: float, right_actuator_power: float, conveyor_power: float, hatch_open: bool):
        """Set the power of the excavator motors."""

        if (abs(left_actuator_power) > 1.0 or abs(right_actuator_power) > 1.0):
            self.get_logger.info(f"actuator motor power must be between -1.0 and 1.0: L({left_actuator_power}), R({right_actuator_power})")
            return

        # ensure motor power between -1.0 and 1.0
        if (abs(conveyor_power) > 1.0):
            self.get_logger.info(f"conveyor motor power must be between -1.0 and 1.0")
            return

        # message class (cannot move and excavate)
        msg = Excavator()
        msg.left_actuator_speed = left_actuator_power
        msg.right_actuator_speed = right_actuator_power
        msg.conveyor = conveyor_power
        msg.hatch = hatch_open

        # then publish it like usual
        self.excavate_pub.publish(msg)

    async def move_excavator(self, target_percent: float):
        # reset the PID controllers
        self.excavator_left_actuator_pid.reset()
        self.excavator_right_actuator_pid.reset()
        tolerance_percent = 0.05
        
        # use pid loop to get to the target percent
        while True:
            # find error for PID
            left_error = target_percent - self.excavator_left_percent
            right_error = target_percent - self.excavator_right_percent
            print(f"error: {left_error}, {right_error}")
            # check if completed
            left_completed = abs(left_error) <= tolerance_percent
            right_completed = abs(right_error) <= tolerance_percent
            if left_completed and right_completed: break
            # update PID
            current_time = self.get_clock().now()
            left_power = self.excavator_left_actuator_pid.update(left_error, current_time)
            right_power = self.excavator_right_actuator_pid.update(right_error, current_time)
            print(f"power: {left_power}, {right_power}")
            # make sure theres at least a little power
            if abs(left_power) < 0.1:
                left_power = 0.8 if left_power > 0 else -0.8
            if abs(right_power) < 0.1:
                right_power = 0.1 if right_power > 0 else -0.1
            # but if we are done, stop the motors
            if left_completed:
                left_power = 0.0
            if right_completed:
                right_power = 0.0
            print(f"power after: {left_power}, {right_power}")
            # drive the motors for a bit
            self.set_excavator_power(left_power, right_power, 0.0, False)
            await self.yield_once(0.2)
        print("done")
        # stop the excavator
        self.stop_excavator()

    async def run_conveyor(self):
        # in position, now turn on conveyor
        self.set_excavator_power(0.0, 0.0, 0.7, False)
        # wait to excavate
        await self.yield_once(10.0)
        # stop the conveyor
        self.stop_excavator()

    def stop_excavator(self):
        self.set_excavator_power(0.0, 0.0, 0.0, False)

    async def run_unload_bin(self):
        # open
        self.set_excavator_power(0.0, 0.0, 0.0, True)
        await self.yield_once(3.0)
        # close
        self.set_excavator_power(0.0, 0.0, 0.0, False)
        
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
            
            if self.was_cancelled: return

        # stop the motors
        self.set_motor_power(0.0, 0.0)
        self.get_logger().info(f"rotated to {int(self.orientation)} degrees. (wanted {int(final_orientation)})")

    async def to_position(self, destination: Point, go_reverse: bool = False):
        """Drives to a given destination with PID control."""
        start_time = self.get_clock().now()
        # calculate orientation to face position
        self.get_logger().info(f"want to go to position {destination.x}, {destination.y}")
        self.website_path_visualizer.publish(PathVisual(nodes=[self.position, destination]))

        # flip robot around if going reverse
        deg_offset = 0.0
        if (go_reverse):
            deg_offset = 180.0

        await self.face_position(destination, deg_offset)
        after_orientation_time = self.get_clock().now()
        
        # reset the PID controllers
        self.linear_drive_pid.reset()
        self.angular_drive_pid.reset()
        
        while True:
            # find error
            error = distance(self.position, destination)
            orientation_error = self.get_orientation_error(destination)

            # reverse error calculation if going backward
            # shift the PID notion of "aligned" by 180.0
            if go_reverse:
                err_orient = (err_orient + 180.0) % (360.0) - 180.0

            # check if completed
            if error < self.dist_tolerance: break
            # update pid
            now = self.get_clock().now()
            forward_power = self.linear_drive_pid.update(error, now)
            turn_power = self.angular_drive_pid.update(orientation_error, now)
            # adjust motor power based on turn power from PID
            left_power = clamp(forward_power - turn_power, -1.0, 1.0)
            right_power = clamp(forward_power + turn_power, -1.0, 1.0)

            if (go_reverse):
                left_power = -left_power
                right_power = -right_power

            await self.drive(left_power, right_power, seconds=0.2)
            await self.yield_once()
            
            if self.was_cancelled: return


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
    async def face_position(self, destination: Point, deg_offset: float = 0.0):
        """Rotates to face a given position."""
        self.get_logger().info(f"want to face {destination.x}, {destination.y}")
        angle_rad = math.atan2(self.position.y - destination.y, destination.x - self.position.x)
        new_orientation = positive_angle(math.degrees(angle_rad))

        # rotate to new orientation
        old_orientation = self.orientation

        # move to face position plus client specified offset
        new_orientation = positive_angle(new_orientation + deg_offset)

        await self.to_orient(new_orientation)
        self.get_logger().info(f"faced position: old {int(old_orientation)}; new {int(self.orientation)}; desired {int(new_orientation)}")

    # navigate along an entire path worth of coordinates (points)
    async def to_path(self, points, goal_handle, feedback_msg, go_reverse: bool):
        """Drives to a series of points in order, while reporting its progress over the goal handle."""
        points_len = len(points)
        
        # loop through each point and go there on the path
        # we enumerate to keep track of iterations
        for i, point in enumerate(points):
            await self.to_position(point, go_reverse)

            # publish progress traveling the path
            # e.g. 1 / 2 meaning 1 point reached out of 2 so far
            feedback_msg.progress = (i + 1) / points_len
            feedback_msg.finished_driving = i + 1 >= points_len
            goal_handle.publish_feedback(feedback_msg)
            await self.yield_once()
            
            if self.was_cancelled:
                return
        
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
