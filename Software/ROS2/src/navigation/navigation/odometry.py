import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


# the self driver agent action
from lunabotics_interfaces.msg import Point, Motors, Excavator, PathVisual, ExcavatorPotentiometer, AccelerometerCorrection, Config
from lunabotics_interfaces.action import SelfDriver
from navigation.pathfinding import distance
from sensors.spin_node_helper import spin_nodes

from navigation.pid import PIDController


from std_msgs.msg import Float32
import math

MAX_ACTUATOR_PERCENT = 0.9
MIN_ACTUATOR_PERCENT = 0.1

# when the excavator lifter motor is at this %, the distance sensor can be used to detect the ground
DISTANCE_SENSOR_ENABLE_THRESHOLD = 0.7
# when the distance sensor is this far away, the excavator should stop and start digging
DISTANCE_SENSOR_TOLERANCE = 8  # cm

MAX_EXCAVATOR_LIFTER_PERCENT = 0.9
MIN_EXCAVATOR_LIFTER_PERCENT = 0.1

CONVEYOR_SPEED = 0.6

EXCAVATION_TIME = 10.0
BIN_TIME = 3.0

class Odometry(Node):
    """
        This node has an action server that takes requests to move the robot.
        It keeps track of the robot's position and orientation and wait until the robot reaches the requested position.
    """
    
    def __init__(self):
        super().__init__('odometry')

        # these make sure each subcriber/action/publisher gets time to run
        self.action_group = ReentrantCallbackGroup()      # For action server
        self.sensor_group = ReentrantCallbackGroup()      # For sensor callbacks
        self.control_group = MutuallyExclusiveCallbackGroup()  # For control outputs
        self.config_group = ReentrantCallbackGroup()      # For configuration

        # action server is used to drive the robot
        # it's in its own callback group so it can run async functions
        self._action_server = ActionServer(
            self,
            SelfDriver,
            'self_driver',
            execute_callback=self.on_goal_request,
            cancel_callback=self.on_cancel,
            callback_group=self.action_group
        )
        
        # action values to store for when its done
        self.goal_handle = None
        self.was_cancelled = False

        # margin of error for rotation (degrees)
        self.deg_tolerance = 3.0 
        # margin of error for movement (centimeters)
        self.dist_tolerance = 15.0
        # margin of error for excavator actuators (percent)
        self.actuator_tolerance_percent = 0.03

        # initalize position
        self.position = Point()
        self.position.x = 448
        self.position.y = 100

        # initalize orientation
        self.orientation = 0.0
        self.last_orientation_time = self.get_clock().now()
        self.has_corrected_orientation = False
        
        # initalize linear actuator percentage for opening the bin
        # 0% is contracted and 100% is extended
        self.actuator_percent = 0.0
        self.excavator_lifter_percent = 0.0
        self.excavator_ground_distance = -1.0  # cm
        
        # ros publishers and subscribers
        # control
        self.motor_pub = self.create_publisher(Motors, 'physical_robot/motors', 10, callback_group=self.control_group)
        self.excavate_pub = self.create_publisher(Excavator, 'physical_robot/excavator', 10, callback_group=self.control_group)
        self.orientation_corrector = self.create_publisher(AccelerometerCorrection, 'sensors/accelerometer_correction', 10, callback_group=self.control_group)
        
        # config
        self.config_sub = self.create_subscription(Config, 'website/config', self.on_config, 10, callback_group=self.config_group)
        self.website_path_visualizer = self.create_publisher(PathVisual, 'navigation/odometry_path', 10, callback_group=self.config_group)
        
        # sensors
        self.position_sub = self.create_subscription(Point, 'sensors/position', self.on_position, 10, callback_group=self.sensor_group)
        self.orientation_sub = self.create_subscription(Float32, 'sensors/orientation', self.on_orientation, 10, callback_group=self.sensor_group)
        self.excavator_distance_sub = self.create_subscription(Float32, 'sensors/excavator_distance', self.on_excavator_distance, 10, callback_group=self.sensor_group)
        self.excavator_percent_sub = self.create_subscription(ExcavatorPotentiometer, 'sensors/excavator_percent', self.on_excavator_percent, 10, callback_group=self.sensor_group)
        
        # PID controllers (some notes from ChatGPT: )
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
        
        self.bin_actuator_pid = PIDController(Kp=0.6, Ki=0.0, Kd=0.001, output_limits=(-1.0, 1.0))
        self.excavator_lifter_pid = PIDController(Kp=0.6, Ki=0.0, Kd=0.001, output_limits=(-1.0, 1.0))

    # when the config is received for tuning pid loop
    def on_config(self, config: Config):
        if config.node != "odometry":
            return
        chosen_pid = None
        match config.category:
            case "orientation_pid":
                chosen_pid = self.orientation_pid
            case "linear_drive_pid":
                chosen_pid = self.linear_drive_pid
            case "angular_drive_pid":
                chosen_pid = self.angular_drive_pid
        if chosen_pid is not None:
            match config.setting:
                case "Kp":
                    chosen_pid.Kp = float(config.value)
                    self.get_logger().info(f"set Kp to {chosen_pid.Kp} on {config.category}")
                case "Ki":
                    chosen_pid.Ki = float(config.value)
                    self.get_logger().info(f"set Ki to {chosen_pid.Ki} on {config.category}")
                case "Kd":
                    chosen_pid.Kd = float(config.value)
                    self.get_logger().info(f"set Kd to {chosen_pid.Kd} on {config.category}")
        
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
        # self.get_logger().info('received goal with {0} points'.format(len(points)))
        
        # correct the orientation (the robot is rotated randomly at the start of the competition)
        if not self.has_corrected_orientation:
            self.get_logger().info('correcting orientation')
            # perform the correction
            await self.perform_orientation_correction()
            self.get_logger().info('corrected orientation')
            # set the flag to true so we dont do it again
            self.has_corrected_orientation = True

        # drive in reverse if told by the goal
        await self.to_path(points, goal_handle, feedback_msg, goal_handle.request.should_reverse)
        
        # get if we should unload
        unload = goal_handle.request.should_unload
        # get if we should excavate
        excavate = goal_handle.request.should_excavate
        if unload:
            self.get_logger().info('unloading')
            await self.move_actuator_to(MAX_ACTUATOR_PERCENT)
            await self.yield_once(delay=BIN_TIME)
            await self.move_actuator_to(MIN_ACTUATOR_PERCENT)
            self.get_logger().info('unloaded')
        elif excavate:
            self.get_logger().info('excavating')
            # move the excavator up
            await self.move_excavator_to(MAX_EXCAVATOR_LIFTER_PERCENT)
            self.get_logger().info('excavator in position')
            # start the conveyor
            await self.run_conveyor()
            self.get_logger().info('conveyor done running')
            # move the excavator down
            await self.move_excavator_to(MIN_EXCAVATOR_LIFTER_PERCENT)
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
        self.set_excavator_percent(percent.excavator_lifter_percent, percent.actuator_percent)
        
    # when excavator distance updates
    def on_excavator_distance(self, distance):
        self.set_excavator_distance(distance.data)

    def set_position(self, position: Point):
        """Called by the ROS subscriber."""
        self.position = position
    
    def set_excavator_distance(self, distance: float):
        # distances far away are not very accurate and not needed
        if distance < 0.0 or distance > 200.0:
            return
        self.excavator_ground_distance = distance
    
    def set_excavator_percent(self, excavator_lifter_percent: float, actuator_percent: float):
        """Called by the ROS subscriber. Enforces percent between 0 and 1."""
        if (excavator_lifter_percent < 0.0 or excavator_lifter_percent > 1.0 and actuator_percent < 0.0 or actuator_percent > 1.0):
            self.get_logger().info(f"invalid excavator percent")
            return
        self.excavator_lifter_percent = excavator_lifter_percent
        self.actuator_percent = actuator_percent

    def set_orientation(self, orientation: float):
        """Called by the ROS subscriber. Enforces orientation between 0 and 360 degrees."""
        if (orientation < 0.0 or orientation > 360.0):
            self.get_logger().info(f"invalid orientation: {orientation}")
            return
        self.orientation = orientation
        
        delta_time = (self.get_clock().now() - self.last_orientation_time).nanoseconds / 1_000_000_000
        hz = 1.0/delta_time
        if hz < 5:
            pass
            # self.get_logger().info(f"lagging! {hz}hz")
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
    def set_excavator_power(self, excavator_lift_power: float, actuator_power: float, conveyor_power: float):
        """Set the power of the excavator motors."""

        if (abs(excavator_lift_power) > 1.0 or abs(actuator_power) > 1.0):
            self.get_logger.info(f"motor power must be between -1.0 and 1.0: L({excavator_lift_power}), R({actuator_power})")
            return

        # ensure motor power between -1.0 and 1.0
        if (abs(conveyor_power) > 1.0):
            self.get_logger.info(f"conveyor motor power must be between -1.0 and 1.0")
            return

        # message class (cannot move and excavate)
        msg = Excavator()
        msg.excavator_lifter_speed = float(excavator_lift_power)
        msg.actuator_speed = float(actuator_power)
        msg.conveyor_speed = float(conveyor_power)

        # then publish it like usual
        self.excavate_pub.publish(msg)
    
    async def move_actuator_to(self, target_percent: float):
        await self.move_excavator_motor_to_target(
            target=target_percent,
            feedback=lambda: self.actuator_percent,
            pid=self.bin_actuator_pid,
            drive_callback=lambda power: self.set_excavator_power(0.0, power, 0.0)
        )
    
    def get_excavator_feedback(self):
        # TODO: use distance sensor when excavator lifter percent is high
        return self.excavator_lifter_percent
    
    async def move_excavator_to(self, target_percent: float):
        await self.move_excavator_motor_to_target(
            target=target_percent,
            feedback=self.get_excavator_feedback,
            pid=self.excavator_lifter_pid,
            drive_callback=lambda power: self.set_excavator_power(power, 0.0, 0.0)
        )

    async def move_excavator_motor_to_target(self, target: float, feedback, pid, drive_callback):
        pid.reset()
        
        # use pid loop to get to the target percent
        while True:
            # find error for PID
            error = target - feedback()
            # check if completed
            completed = abs(error) <= self.actuator_tolerance_percent
            if completed: break
            # update PID
            current_time = self.get_clock().now()
            power = pid.update(error, current_time)
            # make sure theres at least a little power
            if abs(power) < 0.1:
                power = 0.1 if power > 0 else -0.1
            self.get_logger().info(f"power after: {power}")
            # drive the motors for a bit
            drive_callback(power)
            await self.yield_once(0.2)
        # stop the excavator
        self.stop_excavator()

    async def run_conveyor(self):
        # in position, now turn on conveyor
        self.set_excavator_power(0.0, 0.0, CONVEYOR_SPEED)
        # wait to excavate
        await self.yield_once(EXCAVATION_TIME)
        # stop the conveyor
        self.stop_excavator()

    def stop_excavator(self):
        self.set_excavator_power(0.0, 0.0, 0.0)
        
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
        # self.get_logger().info(f"rotated to {int(self.orientation)} degrees. (wanted {int(final_orientation)})")

    async def to_position(self, destination: Point, go_reverse: bool = False):
        """Drives to a given destination with PID control."""
        start_time = self.get_clock().now()
        # calculate orientation to face position
        # self.get_logger().info(f"want to go to position {destination.x}, {destination.y}")
        self.website_path_visualizer.publish(PathVisual(nodes=[self.position, destination]))

        # flip robot around if going reverse
        deg_offset = 0.0
        if (go_reverse):
            deg_offset = 180.0

        # return early if already at the position
        error = distance(self.position, destination)
        if error < self.dist_tolerance: return
        
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
                # self.get_logger().info("CHANGE ORIENTATION ERROR (REVERSE)")
                orientation_error = (orientation_error + 180.0) % (360.0) - 180.0

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
        # self.get_logger().info(f"drove to {int(self.position.x)}, {int(self.position.y)} (wanted {int(destination.x)}, {int(destination.y)})")
        after_moving_time = self.get_clock().now()
        # calculate time taken to move
        time_taken = (after_moving_time - start_time).nanoseconds // 1_000_000
        # self.get_logger().info(f"took {time_taken} milliseconds to move")
        # calculate time taken to orient
        orientation_time = (after_orientation_time - start_time).nanoseconds // 1_000_000
        # self.get_logger().info(f"took {orientation_time} milliseconds to orient")

    # orient the rover to face a position
    async def face_position(self, destination: Point, deg_offset: float = 0.0):
        """Rotates to face a given position."""
        # self.get_logger().info(f"want to face {destination.x}, {destination.y}")
        angle_rad = math.atan2(self.position.y - destination.y, destination.x - self.position.x)
        new_orientation = positive_angle(math.degrees(angle_rad))

        # rotate to new orientation
        old_orientation = self.orientation

        # move to face position plus client specified offset
        new_orientation = positive_angle(new_orientation + deg_offset)
        initial_error = self.get_degrees_error(new_orientation)
        # if we are already facing the position, no need to rotate
        if abs(initial_error) <= 15.0:
            # self.get_logger().info(f"already facing position {int(new_orientation)} degrees")
            return
        await self.to_orient(new_orientation)
        # self.get_logger().info(f"faced position: old {int(old_orientation)}; new {int(self.orientation)}; desired {int(new_orientation)}")

    # navigate along an entire path worth of coordinates (points)
    async def to_path(self, points, goal_handle, feedback_msg, go_reverse: bool):
        """Drives to a series of points in order, while reporting its progress over the goal handle."""
        points_len = len(points)
        # go_reverse = True
        
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
    
    async def perform_orientation_correction(self):
        start_position = self.position
        start_orientation = self.orientation
        # drive forwards a bit
        drive_target = 35  # cm
        while True:
            # check if we have driven far enough
            error = distance(start_position, self.position)
            if error >= drive_target: break
            await self.drive(0.35, 0.35, seconds=0.2)
        self.get_logger().info(f"driven {int(error)} cm to correct orientation")
        self.set_motor_power(0.0, 0.0)
        end_position = self.position
        end_orientation = self.orientation
        # calculate the angle from start to the target
        angle_rad = math.atan2(start_position.y - end_position.y, end_position.x - start_position.x)
        # calculate the difference in orientations (due to drift from driving)
        drift = positive_angle(end_orientation - start_orientation)
        # find corrected orientation
        corrected_orientation = positive_angle(math.degrees(angle_rad) - drift)
        orientation_error = positive_angle(corrected_orientation - start_orientation)
        # corrected_orientation should be close a multiple of 90 degrees
        # find the closest multiple of 90 degrees
        closest_multiple = round(corrected_orientation / 90.0) * 90.0
        self.get_logger().info(f"corrected orientation: {int(corrected_orientation)} degrees (is it close to {int(closest_multiple)}?)")
        # publish correction
        correction = AccelerometerCorrection()
        correction.initial_angle = -orientation_error
        correction.should_reset = False
        self.orientation_corrector.publish(correction)    


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
    spin_nodes(odometry, is_async=True, threads=4)
    

if __name__ == '__main__':
    main()
